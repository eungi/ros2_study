#!/bin/bash
# Authors: Eungi Cho

function os_check()
{
    locale_os_name="${1}"; locale_os_version="${2}"; shift; shift; target_os=("${@}")
    locale_os_version=$(echo ${locale_os_version} | cut -d "." -f1)
    locale_os_version=$(($locale_os_version + 0))
    for value in ${target_os[@]}; do
        target_os_name=$(echo ${value} | cut -d "-" -f1)
        target_os_version=$(echo ${value} | cut -d "-" -f2 | cut -d "." -f1)
        target_os_version=$(($target_os_version + 0))
        if [ ${target_os_name} == ${locale_os_name} ]; then
            if [ ${target_os_version} -le ${locale_os_version} ]; then
                echo "true" && return
            fi
        fi
    done
    echo "false"
}

function cecho()
{
  local exp=$1;
  tput setaf 3;
  tput bold
  echo -e "$exp";
  tput sgr0;
}
