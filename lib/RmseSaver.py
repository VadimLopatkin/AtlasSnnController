# Licensed to the Apache Software Foundation (ASF) under one
# or more contributor license agreements.  See the NOTICE file
# distributed with this work for additional information
# regarding copyright ownership.  The ASF licenses this file
# to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance
# with the License.  You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing,
# software distributed under the License is distributed on an
# "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
# KIND, either express or implied.  See the License for the
# specific language governing permissions and limitations
# under the License.

import csv

class RmseSaver:
    def __init__(self):
        self._rmse_list = []
        print "RmseSaver initialized"

    def save_rmse(self, rmse):
        self._rmse_list.append(rmse)
        if (len(self._rmse_list) % 200) == 0:
            self._save_rmse_csv("/home/vadim/tmp/rmse_bak.csv")

    def write_rmse_csv(self):
        self._save_rmse_csv("/home/vadim/tmp/rmse.csv")

    def _save_rmse_csv(self, filename):
        resultFile = open(filename,'wb')
        wr = csv.writer(resultFile, dialect='excel')
        wr.writerow(self._rmse_list)
        wr.writerow(self._rmse_list[0::2])
        resultFile.close()
        print "RMSE values are saved"