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

class FiringRatesSaver:
    def __init__(self):
        self._firing_rates_list = []

    def save_firing_rates(self, firing_rates):
        self._firing_rates_list.append(firing_rates)
        if (len(self._firing_rates_list) % 100) == 0:
            self._save_firing_rates_csv("/home/vadim/tmp/firing_bak" +
                                        str(len(self._firing_rates_list) /
                                            100) + ".csv")

    def _save_firing_rates_csv(self, filename):
        resultFile = open(filename,'wb')
        wr = csv.writer(resultFile, dialect='excel')
        for row in self._firing_rates_list:
            wr.writerow(row)
        resultFile.close()
        print "Firing rates values are saved"