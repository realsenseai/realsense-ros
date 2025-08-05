#!/bin/bash

# Copyright 2024 RealSense, Inc. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# This script makes sure all files with the following extensions - .h, .hpp, .cpp, .js, .py, .bat, .sh, .txt -
# include the Apache license reference and Intel copyright, as it shown in this file header.
# The script checks also that these files are using Unix line-endings and spaces as delmiters (instead of tabs).
# It is recommended to run this script when adding new files to the project.
# For more info, run ./pr_check.sh --help

set +e

sudo apt-get install dos2unix

ok=0
fixed=0

license_file=$PWD/../LICENSE

year_format="[[:digit:]][[:digit:]][[:digit:]][[:digit:]]"

function check_folder {
    for filename in $(find $1 -type f \( -iname \*.cpp -o -iname \*.h -o -iname \*.hpp -o -iname \*.js -o -iname \*.bat -o -iname \*.sh -o -iname \*.txt -o -iname \*.py \)); do

        # Skip files of 3rd-party libraries which already have their own licenses and copyrights
        if [[ "$filename" == *"importRosbag"* || "$filename" == *"pr_check.sh"* ]]; then
            continue;
        fi

        if [[ $(grep -oP "Software License Agreement" $filename | wc -l) -ne 0 ]]; then
            echo "[WARNING] $filename contains 3rd-party license agreement"
        else
            if [[ ! $filename == *"usbhost"* ]]; then
                # Only check files that are not .gitignore-d
                if [[ $(git check-ignore $filename | wc -l) -eq 0 ]]; then
                    if [[ $(grep -oP "Copyright $year_format RealSense, Inc. All Rights Reserved" $filename | wc -l) -eq 0 &&
                          $(grep -oP "Copyright $year_format-$year_format RealSense, Inc. All Rights Reserved" $filename | wc -l) -eq 0 || 
                          $(grep -oP "Licensed under the Apache License, Version 2.0" $filename | wc -l) -eq 0
                       ]]; then
                        echo "[ERROR] $filename is missing the copyright/license notice"
                        ok=$((ok+1))

                        if [[ $2 == *"fix"* ]]; then
                            # take last 13 linse from LICENSE file, and put them at beginning of $filename
                            if [[ $filename == *".h"* || $filename == *".hpp"* || $filename == *".cpp"* || $filename == *".js"* ]]; then
                                license_str="$(tail -13 ${license_file} | sed -e 's/^   / /' | sed -e 's/^/\/\//')"
                            fi
                            if [[ $filename == *".txt"* || $filename == *".py"* ]]; then
                                license_str="$(tail -13 ${license_file} | sed -e 's/^   / /' | sed -e 's/^/#/')"
                            fi
                            echo "Trying to auto-resolve...";
                            ed -s $filename << END
0i
${license_str}

.
w
q
END
                            fixed=$((fixed+1))
                        fi
                    fi

                    if [[ $(grep -o -P '\t' $filename | wc -l) -ne 0 ]]; then
                        echo "[ERROR] $filename has tabs (this project is using spaces as delimiters)"
                        ok=$((ok+1))

                        if [[ $2 == *"fix"* ]]; then
                            echo "Trying to auto-resolve...";
                            sed -i.bak $'s/\t/    /g' $filename
                            fixed=$((fixed+1))
                        fi
                    fi

                    if [[ $(file ${filename} | grep -o -P 'CRLF' | wc -l) -ne 0 ]]; then
                        echo "[ERROR] $filename is using DOS line endings (this project is using Unix line-endings)"
                        ok=$((ok+1))

                        if [[ $2 == *"fix"* ]]; then
                            echo "Trying to auto-resolve...";
                            dos2unix $filename
                            fixed=$((fixed+1))
                        fi
                    fi
                fi
            fi
        fi
    done
}

if [[ $1 == *"help"* ]]; then
    echo Pull-Request Check tool
    echo "Usage: (run from repo scripts directory)"
    echo "    ./pr_check.sh [--help] [--fix]"
    echo "    --fix    Try to auto-fix defects"
    exit 0
fi


cd ..
check_folder . $1
cd scripts

if [[ ${fixed} -ne 0 ]]; then
    echo "Re-running pr_check..."
    ./pr_check.sh
else
    if [[ ${ok} -ne 0 ]]; then
        echo Pull-Request check failed, please address ${ok} the errors reported above
        exit 1
    fi
fi

exit 0
