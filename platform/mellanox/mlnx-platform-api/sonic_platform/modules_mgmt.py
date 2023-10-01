import threading
import time
import queue
import os
import select
import traceback

try:
    from sonic_py_common.logger import Logger
    from sonic_py_common import device_info, multi_asic
    from .device_data import DeviceDataManager
    from sonic_platform_base.sfp_base import SfpBase
    from sonic_platform_base.sonic_xcvr.fields import consts
    from . import sfp as sfp_module
    from . import utils
    from swsscommon.swsscommon import SonicV2Connector
except ImportError as e:
    raise ImportError (str(e) + "- required module not found")

# Global logger class instance
logger = Logger()

STATE_HW_NOT_PRESENT = "Initial state. module is not plugged to cage."
STATE_HW_PRESENT = "Module is plugged to cage"
STATE_MODULE_AVAILABLE = "Module hw present and power is good"
STATE_POWERED = "Module power is already loaded"
STATE_NOT_POWERED = "Module power is not loaded"
STATE_FW_CONTROL = "The module is not CMIS and FW needs to handle"
STATE_SW_CONTROL = "The module is CMIS and SW needs to handle"
STATE_ERROR_HANDLER = "An error occurred - read/write error, power limit or power cap."
STATE_POWER_LIMIT_ERROR = "The cage has not enough power for the plugged module"
STATE_SYSFS_ERROR = "An error occurred while writing/reading SySFS."

INDEP_PROFILE_FILE = "/{}/independent_mode_support.profile"
SAI_INDEP_MODULE_MODE = "SAI_INDEPENDENT_MODULE_MODE"
SAI_INDEP_MODULE_MODE_DELIMITER = "="
SAI_INDEP_MODULE_MODE_TRUE_STR = "1"
SYSFS_LEGACY_PRESENCE_FD = "/sys/module/sx_core/asic0/module{}/present"
ASIC_NUM = 0
PORT_BREAKOUT = 8
SYSFS_INDEPENDENT_FD_PREFIX_WO_MODULE = "/sys/module/sx_core/asic{}".format(ASIC_NUM)
SYSFS_INDEPENDENT_FD_PREFIX = SYSFS_INDEPENDENT_FD_PREFIX_WO_MODULE + "/module{}"
SYSFS_INDEPENDENT_FD_PRESENCE = '/'.join([SYSFS_INDEPENDENT_FD_PREFIX, "hw_present"])
SYSFS_INDEPENDENT_FD_POWER_GOOD = '/'.join([SYSFS_INDEPENDENT_FD_PREFIX, "power_good"])
SYSFS_INDEPENDENT_FD_POWER_ON = '/'.join([SYSFS_INDEPENDENT_FD_PREFIX, "power_on"])
SYSFS_INDEPENDENT_FD_HW_RESET = '/'.join([SYSFS_INDEPENDENT_FD_PREFIX, "hw_reset"])
SYSFS_INDEPENDENT_FD_POWER_LIMIT = '/'.join([SYSFS_INDEPENDENT_FD_PREFIX, "power_limit"])
SYSFS_INDEPENDENT_FD_FW_CONTROL = '/'.join([SYSFS_INDEPENDENT_FD_PREFIX, "control"])
# echo <val>  /sys/module/sx_core/$asic/$module/frequency   //  val: 0 - up to 400KHz, 1 - up to 1MHz
SYSFS_INDEPENDENT_FD_FREQ = '/'.join([SYSFS_INDEPENDENT_FD_PREFIX, "frequency"])
IS_INDEPENDENT_MODULE = 'is_independent_module'

STATE_DB_TABLE_NAME_PREFIX = 'TRANSCEIVER_MODULES_MGMT|{}'


class ModulesMgmtTask(threading.Thread):
    RETRY_EEPROM_READING_INTERVAL = 60

    def __init__(self, namespaces=None, port_mapping=None, main_thread_stop_event=None, sfp_error_event=None, q=None
                 ,l=None):
        threading.Thread.__init__(self)
        self.name = "SfpStateUpdateTask"
        self.exc = None
        self.task_stopping_event = threading.Event()
        self.main_thread_stop_event = main_thread_stop_event
        self.sfp_error_event = sfp_error_event
        #self.port_mapping = copy.deepcopy(port_mapping)
        # A set to hold those logical port name who fail to read EEPROM
        self.retry_eeprom_set = set()
        # To avoid retry EEPROM read too fast, record the last EEPROM read timestamp in this member
        self.last_retry_eeprom_time = 0
        # A dict to hold SFP error event, for SFP insert/remove event, it is not necessary to cache them
        # because _wrapper_get_presence returns the SFP presence status
        self.sfp_error_dict = {}
        self.sfp_insert_events = {}
        self.sfp_port_dict_initial = {}
        self.sfp_port_dict = {}
        self.sfp_changes_dict = {}
        self.sfp_delete_list_from_port_dict = []
        self.namespaces = namespaces
        self.modules_changes_queue = q
        self.modules_queue_lock = l
        self.is_supported_indep_mods_system = False
        self.modules_lock_list = []
        self.waiting_modules_list = Set()
        self.timer = threading.Thread()
        self.timer_queue = queue.Queue()
        self.timer_queue_lock = threading.Lock()
        self.poll_obj = None
        self.fds_mapping_to_obj = {}

    # SFPs state machine
    def get_sm_func(self, sm, port):
        SFP_SM_ENUM = {STATE_HW_NOT_PRESENT: self.check_if_hw_present
            , STATE_HW_PRESENT: self.checkIfModuleAvailable
            , STATE_MODULE_AVAILABLE: self.checkIfPowerOn
            , STATE_NOT_POWERED: self.powerOnModule
            , STATE_POWERED: self.checkModuleType
            , STATE_FW_CONTROL: self.saveModuleControlMode
            , STATE_SW_CONTROL: self.saveModuleControlMode
            , STATE_ERROR_HANDLER: STATE_ERROR_HANDLER
            , STATE_POWER_LIMIT_ERROR: STATE_POWER_LIMIT_ERROR
            , STATE_SYSFS_ERROR: STATE_SYSFS_ERROR
        }
        print_and_log("getting func for state {} for port {}".format(sm, port))
        try:
            func = SFP_SM_ENUM[sm]
            print_and_log("got func {} for state {} for port {}".format(func, sm, port))
            return func
        except KeyError as e:
            print_and_log("exception {} for port {}".format(e, port))
        return None

    def run(self):
        # check first if the system supports independent mode and set boolean accordingly
        (platform_path, hwsku_dir) = device_info.get_paths_to_platform_and_hwsku_dirs()
        #hwsku = device_info.get_hwsku()
        independent_file = INDEP_PROFILE_FILE.format(hwsku_dir)
        if os.path.isfile(independent_file):
            print_and_log("file {} found, checking content for independent mode value".format(independent_file))
            with open(independent_file, "r") as independent_file_fd:
                independent_file_content = independent_file_fd.read()
                if SAI_INDEP_MODULE_MODE in independent_file_content and \
                        SAI_INDEP_MODULE_MODE_DELIMITER in independent_file_content:
                    independent_file_splitted = independent_file_content.split(SAI_INDEP_MODULE_MODE_DELIMITER)
                    if (len(independent_file_splitted) > 1):
                        self.is_supported_indep_mods_system = int(independent_file_splitted[1]) == int(SAI_INDEP_MODULE_MODE_TRUE_STR)
                        print_and_log("file {} found, system will work in independent mode".format(independent_file))
                        print_and_log("value of indep mode var: {} found in file".format(independent_file_splitted[1]))
        else:
            print_and_log("file {} not found, system stays in legacy mode".format(independent_file))

        # static init - at first go over all ports and check each one if it's independent module or legacy
        self.sfp_changes_dict = {}
        # check for each port if the module connected and if it supports independent mode or legacy
        num_of_ports = DeviceDataManager.get_sfp_count()
        # create the modules sysfs fds poller
        #self.poll_obj = select.poll()
        self.poll_obj = []
        for port in range(num_of_ports):
            #temp_port_dict = {IS_INDEPENDENT_MODULE: False}
            # check sysfs per port whether it's independent mode or legacy
            temp_module_sm = ModuleStateMachine(port_num=port, initial_state=STATE_HW_NOT_PRESENT
                                              , current_state=STATE_HW_NOT_PRESENT)
            module_fd_indep_path = SYSFS_INDEPENDENT_FD_PRESENCE.format(port)
            print_and_log("system in indep mode: {} port {}".format(self.is_supported_indep_mods_system, port))
            if self.is_supported_indep_mods_system and os.path.isfile(module_fd_indep_path):
                print_and_log("system in indep mode: {} port {} reading file {}".format(self.is_supported_indep_mods_system, port, module_fd_indep_path))
                temp_module_sm.set_is_indep_modules(True)
                temp_module_sm.set_module_fd_path(module_fd_indep_path)
                module_fd = open(module_fd_indep_path, "r")
                temp_module_sm.set_module_fd(module_fd)
            else:
                module_fd_legacy_path = SYSFS_LEGACY_PRESENCE_FD.format(port)
                temp_module_sm.set_module_fd_path(module_fd_legacy_path)
                module_fd = open(module_fd_legacy_path, "r")
                temp_module_sm.set_module_fd(module_fd)
            # add lock to use with timer task updating next state per module object
            self.modules_lock_list.append(threading.Lock())
            # register the module's sysfs fd to poller with ERR and PRI attrs
            #self.poll_obj.register(module_fd, select.POLLERR | select.POLLPRI)
            #self.fds_mapping_to_obj[module_fd.fileno()] = temp_module_sm
            #temp_module_sm.set_poll_obj(self.poll_obj)
            # start SM for this independent module
            print_and_log("adding temp_module_sm {} to sfp_port_dict".format(temp_module_sm))
            self.sfp_port_dict_initial[port] = temp_module_sm
            self.sfp_port_dict[port] = temp_module_sm

        i = 0
        # need at least 1 module in final state until it makes sense to send changes dict
        is_final_state_module = False
        all_static_detection_done = False
        print_and_log("sfp_port_dict before starting static detection: {}".format(self.sfp_port_dict))
        # loop on different state for ports in static detection until all done
        while (not self.task_stopping_event or not self.main_thread_stop_event) and not all_static_detection_done:
            print_and_log("static detection running iteration {}".format(i))
            waiting_list_len = len(self.waiting_modules_list)
            sfp_port_dict_keys_len = len(self.sfp_port_dict.keys())
            # if all ports in waiting list - sleep one second rather than looping over and over again on same state
            if waiting_list_len == sfp_port_dict_keys_len:
                print_and_log("static detection length of waiting list {} and sfp port dict keys {} is the same, sleeping 1 second..."
                              .format(waiting_list_len, sfp_port_dict_keys_len))
                time.sleep(1)
            else:
                print_and_log("static detectionlength of waiting list {} and sfp port dict keys {} is different, NOT sleeping 1 second"
                              .format(waiting_list_len, sfp_port_dict_keys_len))
            for port_num, module_sm_obj in self.sfp_port_dict.items():
                curr_state = module_sm_obj.get_current_state()
                print_and_log(f'static detection STATE_LOG {port_num}: curr_state is {curr_state}')
                func = self.get_sm_func(curr_state, port_num)
                print_and_log("static detectiongot returned func {} for state {}".format(func, curr_state))
                try:
                    if not isinstance(func, str):
                        next_state = func(port_num, module_sm_obj)
                except TypeError as e:
                    print_and_log("static detection exception {} for port {} traceback:\n{}".format(e, port_num, traceback.format_exc()))
                    continue
                print_and_log(f'static detection STATE_LOG {port_num}: next_state is {next_state}')
                if self.timer.is_alive():
                    #print_and_log("static detection timer threads is alive, acquiring lock")
                    self.modules_lock_list[port_num].acquire()
                # for STATE_NOT_POWERED we dont advance to next state, timerTask is doing it into STATE_POWERED
                if curr_state != STATE_NOT_POWERED or not module_sm_obj.wait_for_power_on:
                    module_sm_obj.set_next_state(next_state)
                    module_sm_obj.advance_state()
                if module_sm_obj.get_final_state():
                    print_and_log(f'static detection STATE_LOG {port_num}: enter final state {module_sm_obj.get_final_state()}')
                    is_final_state_module = True
                if self.timer.is_alive():
                    self.modules_lock_list[port_num].release()
                is_timer_alive = self.timer.is_alive()
                #print_and_log("static detection timer thread is_alive {} port {}".format(is_timer_alive, port_num))
                if STATE_NOT_POWERED == curr_state:
                    if not is_timer_alive:
                        print_and_log ("static detection curr_state is {} and timer thread is_alive {}, running timer task thread"
                               .format(curr_state, is_timer_alive))
                        # call timer task
                        self.timer = threading.Timer(1.0, self.timerTask)
                        self.timer.start()
                    self.timer_queue.put(module_sm_obj)
                    if self.timer.is_alive():
                        print_and_log("timer thread is_alive {}, locking module obj".format(self.timer.is_alive()))
                        self.modules_lock_list[port_num].acquire()
                    module_sm_obj.set_next_state(next_state)
                    if self.timer.is_alive():
                        print_and_log("timer thread is_alive {}, releasing module obj".format(self.timer.is_alive()))
                        self.modules_lock_list[port_num].release()

            state_db = None
            for port, module_obj in self.sfp_port_dict_initial.items():
                final_state = module_obj.get_final_state()
                if port in self.sfp_port_dict.keys() and final_state:
                    self.sfp_changes_dict[str(module_obj.port_num)] = '0' if final_state in [STATE_HW_NOT_PRESENT, STATE_ERROR_HANDLER] else '1'
                    if final_state in [STATE_SW_CONTROL, STATE_FW_CONTROL]:
                        namespaces = multi_asic.get_front_end_namespaces()
                        for namespace in namespaces:
                            print_and_log("static detection getting state_db for port {} namespace {}".format(port, namespace))
                            state_db = SonicV2Connector(use_unix_socket_path=False, namespace=namespace)
                            print_and_log("static detection got state_db for port {} namespace {}".format(port, namespace))
                            if state_db is not None:
                                print_and_log("static detection connecting to state_db for port {} namespace {}".format(port, namespace))
                                state_db.connect(state_db.STATE_DB)
                                if final_state in [STATE_FW_CONTROL]:
                                    control_type = 'FW_CONTROL'
                                elif final_state in [STATE_SW_CONTROL]:
                                    control_type = 'SW_CONTROL'
                                table_name = STATE_DB_TABLE_NAME_PREFIX.format(port)
                                print_and_log("static detection setting state_db table {} for port {} namespace {} control_type {}"
                                              .format(table_name, port, namespace, control_type))
                                state_db.set(state_db.STATE_DB, table_name, "control type", control_type)
                    del self.sfp_port_dict[port]

            if len(self.sfp_changes_dict) > 0:
                print_and_log("static detection putting sfp_changes_dict {} in modules changes queue..."
                              .format(self.sfp_changes_dict))
                try:
                    self.modules_queue_lock.acquire()
                    self.modules_changes_queue.put(self.sfp_changes_dict, timeout=1)
                    self.modules_queue_lock.release()
                    self.sfp_changes_dict = {}
                except queue.Full:
                    print_and_log("failed to put item from modules changes queue, queue is full")
            else:
                print_and_log("static detection sfp_changes_dict {} is empty...".format(self.sfp_changes_dict))
            i += 1
            print_and_log("sfp_port_dict: {}".format(self.sfp_port_dict))
            for port_num, module_sm_obj in self.sfp_port_dict.items():
                print_and_log("static detection port_num: {} initial state: {} current_state: {} next_state: {}"
                       .format(port_num, module_sm_obj.initial_state, module_sm_obj.get_current_state()
                               , module_sm_obj.get_next_state()))
            sfp_port_dict_keys_len = len(self.sfp_port_dict.keys())
            if sfp_port_dict_keys_len == 0:
                print_and_log("static detection len of keys of sfp_port_dict is 0: {}".format(sfp_port_dict_keys_len))
                all_static_detection_done = True
            else:
                print_and_log("static detection len of keys of sfp_port_dict is not 0: {}".format(sfp_port_dict_keys_len))
            print_and_log("static detection all_static_detection_done: {}".format(all_static_detection_done))


        print_and_log("sfp_port_dict before dynamic detection: {}".format(self.sfp_port_dict))
        # loop on listening to changes, gather and put them into shared queue, then continue looping
        i = 0
        # need at least 1 module in final state until it makes sense to send changes dict
        is_final_state_module = False
        while not self.task_stopping_event or not self.main_thread_stop_event:
            print_and_log("dynamic detection running iteration {}".format(i))
            # poll for changes with 1 second timeout
            #fds_events = self.poll_obj.poll(1000)
            fds_events = select.select(self.poll_obj, [], [], 1000)
            print_and_log("dynamic detection polled obj checking fds_events iteration {}".format(i))
            for fd, event in fds_events:
                # get modules object from fd according to saved key-value of fd-module obj saved earlier
                print_and_log("dynamic detection working on fd {} event {}".format(fd, event))
                module_obj = self.fds_mapping_to_obj[fd]
                print_and_log("dynamic detection got module_obj {} with port {} from fd number {} path {}"
                              .format(module_obj, module_obj.port_num, fd, module_obj.module_fd_path))
                # put again module obj in sfp_port_dict so next loop will work on it
                module_obj.reset_all_states()
                self.sfp_port_dict[module_obj.port_num] = module_obj
                # put port number in changes dict to pass back to xcvrd's calling SfpStateUpdateTask thread
                #self.sfp_changes_dict[module_obj.port_num] = module_obj

            for port_num, module_sm_obj in self.sfp_port_dict.items():
                curr_state = module_sm_obj.get_current_state()
                print_and_log(f'dynamic detection STATE_LOG {port_num}: curr_state is {curr_state}')
                func = self.get_sm_func(curr_state, port)
                print_and_log("dynamic detection got returned func {} for state {}".format(func, curr_state))
                try:
                    next_state = func(port_num, module_sm_obj)
                except TypeError as e:
                    print_and_log("dynamic detection exception {} for port {}".format(e, port_num))
                    continue
                print_and_log(f'dynamic detection STATE_LOG {port_num}: next_state is {next_state}')
                if self.timer.is_alive():
                    #print_and_log("dynamic detection timer threads is alive, acquiring lock")
                    self.modules_lock_list[port_num].acquire()
                if curr_state != STATE_NOT_POWERED or not module_sm_obj.wait_for_power_on:
                    module_sm_obj.set_next_state(next_state)
                    module_sm_obj.advance_state()
                if module_sm_obj.get_final_state():
                    #print_and_log(f'dynamic detection STATE_LOG {port_num}: enter final state {module_sm_obj.get_final_state()}')
                    is_final_state_module = True
                if self.timer.is_alive():
                    self.modules_lock_list[port_num].release()
                is_timer_alive = self.timer.is_alive()
                #print_and_log("dynamic detection timer thread is_alive {} port {}".format(is_timer_alive, port_num))
                if STATE_NOT_POWERED == curr_state:
                    if not is_timer_alive:
                        print_and_log("dynamic detection curr_state is {} and timer thread is_alive {}, running timer task thread"
                                      .format(curr_state, is_timer_alive))
                        # call timer task
                        self.timer = threading.Timer(1.0, self.timerTask)
                        self.timer.start()
                    self.timer_queue.put(module_sm_obj)
                    if self.timer.is_alive():
                        print_and_log("dynamic detection timer thread is_alive {}, locking module obj".format(self.timer.is_alive()))
                        self.modules_lock_list[port_num].acquire()
                    module_sm_obj.set_next_state(next_state)
                    if self.timer.is_alive():
                        print_and_log(
                            "dynamic detection timer thread is_alive {}, releasing module obj".format(self.timer.is_alive()))
                        self.modules_lock_list[port_num].release()

            if is_final_state_module:
                state_db = None
                for port, module_obj in self.sfp_port_dict.items():
                    final_state = module_obj.get_final_state()
                    if final_state:
                        #del self.sfp_port_dict[port]
                        # add port to delete list that we will iterate on later and delete the ports from sfp_port_dict
                        self.sfp_delete_list_from_port_dict.append(port)
                        self.sfp_changes_dict[str(module_obj.port_num)] = '0' if final_state in [STATE_HW_NOT_PRESENT,
                                                                                                 STATE_ERROR_HANDLER] else '1'
                        if final_state in [STATE_SW_CONTROL, STATE_FW_CONTROL]:
                            namespaces = multi_asic.get_front_end_namespaces()
                            for namespace in namespaces:
                                print_and_log("dynamic detection getting state_db for port {} namespace {}".format(port, namespace))
                                state_db = SonicV2Connector(use_unix_socket_path=False, namespace=namespace)
                                print_and_log("dynamic detection got state_db for port {} namespace {}".format(port, namespace))
                                if state_db is not None:
                                    print_and_log(
                                        "dynamic detection connecting to state_db for port {} namespace {}".format(port, namespace))
                                    state_db.connect(state_db.STATE_DB)
                                    if final_state in [STATE_FW_CONTROL]:
                                        control_type = 'FW_CONTROL'
                                    elif final_state in [STATE_SW_CONTROL]:
                                        control_type = 'SW_CONTROL'
                                    table_name = STATE_DB_TABLE_NAME_PREFIX.format(port)
                                    print_and_log(
                                        "dynamic detection setting state_db table {} for port {} namespace {} control_type {}"
                                        .format(table_name, port, namespace, control_type))
                                    state_db.set(state_db.STATE_DB, table_name,"control type", control_type)

                print_and_log("dynamic detection sfp_port_dict before deletion: {}".format(self.sfp_port_dict))
                for port in self.sfp_delete_list_from_port_dict:
                    print_and_log("dynamic detection deleting port {} from sfp_port_dict".format(port))
                    del self.sfp_port_dict[port]
                self.sfp_delete_list_from_port_dict = []
                print_and_log("dynamic detection sfp_port_dict after deletion: {}".format(self.sfp_port_dict))
                if self.sfp_changes_dict:
                    print_and_log("dynamic detection putting sfp_changes_dict {} in modules changes queue...".format(self.sfp_changes_dict))
                    try:
                        self.modules_queue_lock.acquire()
                        self.modules_changes_queue.put(self.sfp_changes_dict, timeout=1)
                        self.modules_queue_lock.release()
                        self.sfp_changes_dict = {}
                    except queue.Full:
                        print_and_log("failed to put item from modules changes queue, queue is full")
                else:
                    print_and_log("sfp_changes_dict {} is empty...".format(self.sfp_changes_dict))
            i += 1
            print_and_log("sfp_port_dict: {}".format(self.sfp_port_dict))
            for port_num, module_sm_obj in self.sfp_port_dict.items():
                print_and_log("port_num: {} module_sm_obj initial state: {} current_state: {} next_state: {}"
                       .format(port_num, module_sm_obj.initial_state, module_sm_obj.get_current_state(), module_sm_obj.get_next_state()))


    def check_if_hw_present(self, port, module_sm_obj):
        module_fd_indep_path = SYSFS_INDEPENDENT_FD_PRESENCE.format(port)
        if os.path.isfile(module_fd_indep_path):
            try:
                val_int = utils.read_int_from_file(module_fd_indep_path)
                if 0 == val_int:
                    print_and_log("returning {} for val {}".format(STATE_HW_NOT_PRESENT, val_int))
                    module_sm_obj.set_final_state(STATE_HW_NOT_PRESENT)
                    return STATE_HW_NOT_PRESENT
                elif 1 == val_int:
                    print_and_log("returning {} for val {}".format(STATE_HW_PRESENT, val_int))
                    return STATE_HW_PRESENT
            except Exception as e:
                print_and_log("exception {} for port {}, setting as final state STATE_ERROR_HANDLER".format(e, port))
                module_sm_obj.set_final_state(STATE_ERROR_HANDLER)
                return STATE_ERROR_HANDLER
        return STATE_HW_NOT_PRESENT

    def checkIfModuleAvailable(self, port, module_sm_obj):
        print_and_log("enter check_if_module_available port {} module_sm_obj {}".format(port, module_sm_obj))
        module_fd_indep_path = SYSFS_INDEPENDENT_FD_POWER_GOOD.format(port)
        if os.path.isfile(module_fd_indep_path):
            try:
                val_int = utils.read_int_from_file(module_fd_indep_path)
                if 0 == val_int:
                    print_and_log(f'port {port} power is not good')
                    return STATE_HW_NOT_PRESENT
                elif 1 == val_int:
                    print_and_log(f'port {port} power is good')
                #elif 2 == val_int:
                    #self.poll_obj.register(module_sm_obj.module_fd, select.POLLERR | select.POLLPRI)
                    self.poll_obj.append(module_sm_obj.module_fd)
                    self.fds_mapping_to_obj[module_sm_obj.module_fd.fileno()] = module_sm_obj
                    return STATE_MODULE_AVAILABLE
            except Exception as e:
                print_and_log("exception {} for port {}".format(e, port))
                return STATE_HW_NOT_PRESENT
        print_and_log(f'port {port} has no power good file {module_fd_indep_path}')
        return STATE_HW_NOT_PRESENT

    def checkIfPowerOn(self, port, module_sm_obj):
        print_and_log(f'enter checkIfPowerOn for port {port}')
        module_fd_indep_path = SYSFS_INDEPENDENT_FD_POWER_ON.format(port)
        if os.path.isfile(module_fd_indep_path):
            try:
                with open(module_fd_indep_path, "r") as module_fd:
                    val = module_fd.read()
                    val_int = int(val)
                    if 0 == val_int:
                        print_and_log(f'port {port} is not powered')
                        return STATE_NOT_POWERED
                    elif 1 == val_int:
                        if not module_sm_obj.wait_for_power_on and \
                                utils.read_int_from_file(SYSFS_INDEPENDENT_FD_HW_RESET.format(port)) == 1:
                            sfp = sfp_module.SFP(port)
                            xcvr_api = sfp.get_xcvr_api()
                            # only if xcvr_api is None or if it is not active optics cables need reset
                            if not xcvr_api or xcvr_api.is_flat_memory():
                                print_and_log(f'port {port} is powered, but need reset')
                                utils.write_file(SYSFS_INDEPENDENT_FD_HW_RESET.format(port), 0)
                                module_sm_obj.reset_start_time = time.time()
                                module_sm_obj.wait_for_power_on = True
                                utils.write_file(SYSFS_INDEPENDENT_FD_HW_RESET.format(port), 1)
                                module_sm_obj.reset_start_time = time.time()
                                module_sm_obj.wait_for_power_on = True
                                self.waiting_modules_list.append(module_sm_obj)
                                return STATE_NOT_POWERED
                        print_and_log(f'port {port} is powered, does not need reset')
                        return STATE_POWERED
            except Exception as e:
                print_and_log(f'got exception {e} in checkIfPowerOn')
                return STATE_HW_NOT_PRESENT

    def powerOnModule(self, port, module_sm_obj):
        #if module_sm_obj not in self.waiting_modules_list:
        if not module_sm_obj.wait_for_power_on:
            module_fd_indep_path_po = SYSFS_INDEPENDENT_FD_POWER_ON.format(port)
            module_fd_indep_path_r = SYSFS_INDEPENDENT_FD_HW_RESET.format(port)
            try:
                if os.path.isfile(module_fd_indep_path_po):
                    print_and_log("powerOnModule powering on via {} for port {}".format(module_fd_indep_path_po, port))
                    # echo 1 > /sys/module/sx_core/$asic/$module/power_on
                    with open(module_fd_indep_path_po, "w") as module_fd:
                        module_fd.write("1")
                if os.path.isfile(module_fd_indep_path_r):
                    print_and_log("powerOnModule resetting via {} for port {}".format(module_fd_indep_path_r, port))
                    # echo 0 > /sys/module/sx_core/$asic/$module/hw_reset
                    with open(module_fd_indep_path_r, "w") as module_fd:
                        module_fd.write("0")
                module_sm_obj.reset_start_time = time.time()
                module_sm_obj.wait_for_power_on = True
                self.waiting_modules_list.append(module_sm_obj)
            except Exception as e:
                print_and_log("exception in powerOnModule {} for port {}".format(e, port))
                return STATE_HW_NOT_PRESENT
        return STATE_NOT_POWERED

    def checkModuleType(self, port, module_sm_obj):
        print_and_log("enter checkModuleType port {} module_sm_obj {}".format(port, module_sm_obj))
        sfp = sfp_module.SFP(port)
        xcvr_api = sfp.get_xcvr_api()
        if not xcvr_api:
            print_and_log("checkModuleType calling sfp reinit for port {} module_sm_obj {}".format(port, module_sm_obj))
            sfp.reinit()
            print_and_log("checkModuleType setting as FW control as xcvr_api is empty for port {} module_sm_obj {}".format(port, module_sm_obj))
            return STATE_FW_CONTROL
        field = xcvr_api.xcvr_eeprom.mem_map.get_field(consts.ID_FIELD)
        module_type_ba = xcvr_api.xcvr_eeprom.reader(field.get_offset(), field.get_size())
        #module_type = xcvr_api.xcvr_eeprom.read_raw(consts.ID_FIELD)
        if module_type_ba is None:
            print_and_log("checkModuleType module_type is None for port {} - checking if we didnt retry yet".format(port))
            # if we didnt do this retry yet - do it once
            if not module_sm_obj.eeprom_poweron_reset_retry:
                print_and_log("checkModuleType module_type is None retrying by falling back to STATE_NOT_POWERED"
                              "for port {}".format(port))
                module_sm_obj.eeprom_poweron_reset_retry = True
                self.add_port_to_wait_reset(module_sm_obj)
                return STATE_NOT_POWERED
            else:
                print_and_log("checkModuleType module_type is None and already retried - setting as STATE_ERROR_HANDLER"
                              "for port {}".format(port))
                return STATE_ERROR_HANDLER
        module_type = int.from_bytes(module_type_ba, "big")
        print_and_log("checkModuleType got module_type {} in check_module_type port {}".format(port, module_type))
        if not 24 == module_type:
            print_and_log("check_module_type port {} setting STATE_FW_CONTROL due to module ID {}".format(port, module_type))
            module_sm_obj.set_final_state = STATE_FW_CONTROL
            #power_cap = self.checkPowerCapNonCMIS(port, module_sm_obj)
            return STATE_FW_CONTROL
        else:
            if xcvr_api.is_flat_memory():
                print_and_log("check_module_type port {} setting STATE_FW_CONTROL module ID {} due to flat_mem device"
                              .format(module_type, port))
                return STATE_FW_CONTROL
            print_and_log("checking power cap for {} in check_module_type port {} module_sm_obj {}".format(module_type, port, module_sm_obj))
            power_cap = self.checkPowerCap(port, module_sm_obj)
            if power_cap is STATE_POWER_LIMIT_ERROR:
                return STATE_POWER_LIMIT_ERROR
            else:
                # read the module maximum supported clock of Management Comm Interface (MCI) from module EEPROM.
                read_mci = "mci"
                # Then, set it to frequency Sysfs using:
                # echo <val>  /sys/module/sx_core/$asic/$module/frequency   //  val: 0 - up to 400KHz, 1 - up to 1MHz
                #indep_fd_freq = SYSFS_INDEPENDENT_FD_FREQ.format(port)
                #with open(indep_fd_freq, "r") as freq_fd:
                #    freq_fd.write(read_mci)
                return STATE_SW_CONTROL

    def checkPowerCap(self, port, module_sm_obj):
        print_and_log("enter checkPowerCap port {} module_sm_obj {}".format(port, module_sm_obj))
        #sfp_base_module = SfpBase()
        sfp = sfp_module.SFP(port)
        xcvr_api = sfp.get_xcvr_api()
        field = xcvr_api.xcvr_eeprom.mem_map.get_field(consts.MAX_POWER_FIELD)
        powercap_ba = xcvr_api.xcvr_eeprom.reader(field.get_offset(), field.get_size())
        print_and_log("checkPowerCap got powercap bytearray {} for port {} module_sm_obj {}".format(powercap_ba, port, module_sm_obj))
        powercap = int.from_bytes(powercap_ba, "big")
        print_and_log("checkPowerCap got powercap {} for port {} module_sm_obj {}".format(powercap, port, module_sm_obj))
        indep_fd_power_limit = self.get_sysfs_ethernet_port_fd(SYSFS_INDEPENDENT_FD_POWER_LIMIT, port)
        with open(indep_fd_power_limit, "r") as power_limit_fd:
            cage_power_limit = power_limit_fd.read()
        print_and_log("checkPowerCap got cage_power_limit {} for port {} module_sm_obj {}".format(cage_power_limit, port, module_sm_obj))
        if powercap > int(cage_power_limit):
            print_and_log("checkPowerCap powercap {} != cage_power_limit {} for port {} module_sm_obj {}".format(powercap, cage_power_limit, port, module_sm_obj))
            module_sm_obj.set_final_state(STATE_POWER_LIMIT_ERROR)
            return STATE_POWER_LIMIT_ERROR

    def checkPowerCapNonCMIS(self, port, module_sm_obj):
        print_and_log("enter checkPowerCapNonCMIS port {} module_sm_obj {}".format(port, module_sm_obj))
        sfp = sfp_module.SFP(port)
        xcvr_api = sfp.get_xcvr_api()
        serial_id = xcvr_api.xcvr_eeprom.read(consts.SERIAL_ID_FIELD)
        if serial_id is None:
            return None

        ext_id = serial_id[consts.EXT_ID_FIELD]
        power_class = ext_id[consts.POWER_CLASS_FIELD]
        clei_code = ext_id[consts.CLEI_CODE_FIELD]
        cdr_tx = ext_id[consts.CDR_TX_FIELD]
        cdr_rx = ext_id[consts.CDR_RX_FIELD]
        print_and_log("checkPowerCapNonCMIS got powercap {} for port {} module_sm_obj {} clei {} cdr_tx {} cdr_rx {}"
              .format(power_class, port, module_sm_obj, clei_code, cdr_tx, cdr_rx))
        field = xcvr_api.xcvr_eeprom.mem_map.get_field(consts.EXT_ID_FIELD)
        powercap_ba = xcvr_api.xcvr_eeprom.read_raw(field.get_offset(), field.get_size())
        print_and_log("checkPowerCapNonCMIS got powercap bytearray {} for port {} module_sm_obj {}"
                      .format(powercap_ba, port, module_sm_obj))
        powercap = int.from_bytes(powercap_ba, "big") if type(powercap_ba) is bytearray else powercap_ba
        print_and_log("checkPowerCapNonCMIS got powercap {} for port {} module_sm_obj {}"
                      .format(powercap, port, module_sm_obj))


    def saveModuleControlMode(self, port, module_sm_obj):
        print_and_log("saveModuleControlMode setting current state {} for port {} as final state"
                      .format(module_sm_obj.get_current_state(), port))
        # bug - need to find root cause and fix
        #module_sm_obj.set_final_state(module_sm_obj.get_current_state())
        state = module_sm_obj.get_current_state()
        module_sm_obj.final_state = state
        if state == STATE_FW_CONTROL:
            #"echo 0 > /sys/module/sx_core/$asic/$module/control"
            indep_fd_fw_control = SYSFS_INDEPENDENT_FD_FW_CONTROL.format(port)
            with open(indep_fd_fw_control, "w") as fw_control_fd:
                fw_control_fd.write("0")
            print_and_log("saveModuleControlMode set FW control for state {} port {}".format(state, port))
            module_fd_legacy_path = SYSFS_LEGACY_PRESENCE_FD.format(port)
            module_sm_obj.set_module_fd_path(module_fd_legacy_path)
            module_fd = open(module_fd_legacy_path, "r")
            module_sm_obj.set_module_fd(module_fd)
            print_and_log("saveModuleControlMode changed module fd to legacy present for port {}".format(port))
        print_and_log("saveModuleControlMode registering sysfs fd {} number {} path {} for port {}".format(
            module_sm_obj.module_fd, module_sm_obj.module_fd.fileno(), module_sm_obj.set_module_fd_path, port))
        self.poll_obj.register(module_sm_obj.module_fd, select.POLLERR | select.POLLPRI)
        self.fds_mapping_to_obj[module_sm_obj.module_fd.fileno()] = module_sm_obj
        module_sm_obj.set_poll_obj(self.poll_obj)
        print_and_log("saveModuleControlMode set current state {} for port {} as final state {}".format(
            module_sm_obj.get_current_state(), port, module_sm_obj.get_final_state()))

    def STATE_ERROR_HANDLER(self):
        pass

    def STATE_POWER_LIMIT_ERROR(self):
        pass

    def STATE_SYSFS_ERROR(self):
        pass

    def timerTask(self): # wakes up every 1 second
        empty = False
        while not empty:
            empty = True
            for module in self.waiting_modules_list:
                print_and_log("timerTask working on module {}".format(module))
                empty = False
                state = module.get_current_state()
                if module and state == STATE_NOT_POWERED:
                    print_and_log("timerTask module {} current_state {} counting seconds since reset_start_time"
                                  .format(module, module.get_current_state()))
                    if time.time() - module.reset_start_time >= 3:
                        # set next state as STATE_POWERED state to trigger the function of check module type
                        print_and_log("timerTask module port {} locking lock of port {}".format(module.port_num, module.port_num))
                        self.modules_lock_list[module.port_num].acquire()
                        print_and_log("timerTask module port {} setting next state to STATE_POWERED".format(module.port_num))
                        module.set_next_state(STATE_POWERED)
                        print_and_log("timerTask module port {} advancing next state".format(module.port_num))
                        module.advance_state()
                        print_and_log("timerTask module {} releasing lock of port {}".format(module, module.port_num))
                        self.modules_lock_list[module.port_num].release()
                        print_and_log("timerTask module port {} removing module from waiting_modules_list".format(module.port_num))
                        self.waiting_modules_list.remove(module)
            time.sleep(1)

    def get_sysfs_netdev_legacy_ethernet_port_fd(self, sysfs_fd, port):
        breakout_port = "Ethernet{}".format(port * PORT_BREAKOUT)
        sysfs_eth_port_fd = sysfs_fd.format(breakout_port)
        return sysfs_eth_port_fd

    def get_sysfs_ethernet_port_fd(self, sysfs_fd, port):
        sysfs_eth_port_fd = sysfs_fd.format(port)
        return sysfs_eth_port_fd

    def add_port_to_wait_reset(self, module_sm_obj):
        module_sm_obj.reset_start_time = time.time()
        module_sm_obj.wait_for_power_on = True
        self.waiting_modules_list.append(module_sm_obj)



class ModuleStateMachine(object):

    def __init__(self, port_num=0, initial_state=STATE_HW_NOT_PRESENT, current_state=STATE_HW_NOT_PRESENT
                 , next_state=STATE_HW_NOT_PRESENT, final_state='', is_indep_module=False
                 , module_fd_path='', module_fd=None, poll_obj=None, reset_start_time=None
                 , eeprom_poweron_reset_retry=False):
        self.port_num = port_num
        self.initial_state = initial_state
        self.current_state = current_state
        self.next_state = next_state
        self.final_state = final_state
        self.is_indep_modules = is_indep_module
        self.module_fd_path = module_fd_path
        self.module_fd = module_fd
        self.poll_obj = poll_obj
        self.reset_start_time = reset_start_time
        self.wait_for_power_on = False
        self.eeprom_poweron_reset_retry = eeprom_poweron_reset_retry

    def set_initial_state(self, state):
        self.initial_state = state

    def get_current_state(self):
        return self.current_state

    def set_current_state(self, state):
        self.current_state = state

    def get_next_state(self):
        return self.next_state

    def set_next_state(self, state):
        self.next_state = state

    def get_final_state(self):
        return self.final_state

    def set_final_state(self, state):
        self.final_state = state

    def advance_state(self):
        self.set_current_state(self.next_state)
        self.next_state = ''

    def set_is_indep_modules(self, is_indep_modules):
        self.is_indep_modules = is_indep_modules

    def set_module_fd_path(self, module_fd_path):
        self.module_fd_path = module_fd_path

    def set_module_fd(self, module_fd):
        self.module_fd = module_fd

    def get_poll_obj(self):
        return self.poll_obj

    def set_poll_obj(self, poll_obj):
        self.poll_obj = poll_obj

    def reset_all_states(self, def_state=STATE_HW_NOT_PRESENT):
        self.initial_state = def_state
        self.current_state = def_state
        self.next_state = def_state
        self.final_state = ''


def print_and_log(msg):
    logger.log_info(msg)
    print(msg)
