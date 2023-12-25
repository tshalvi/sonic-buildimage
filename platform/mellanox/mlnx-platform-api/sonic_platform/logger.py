#
# Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES.
# Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

import os
from sonic_py_common.logger import Logger

# Global logger instance for nvidia platform API, the argument "enable_set_log_level_on_fly"
# will start a thread to detect CONFIG DB LOGGER table change. The logger instance 
# allow user to set log level via swssloglevel command at real time. This instance 
# should be shared by all modules of platform API to avoid starting too many logger thread.
if os.environ.get("PLATFORM_API_UNIT_TESTING") != "1":
    logger = Logger(log_identifier='nvidia-platform-api', enable_set_log_level_on_fly=True)
else:
    # for unit test, there is no redis, don't set enable_set_log_level_on_fly=True
    logger = Logger(log_identifier='nvidia-platform-api')
