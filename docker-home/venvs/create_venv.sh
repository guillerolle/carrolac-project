#!/usr/bin/bash

path=$(hostname)/$(id --user --name)-python310.venv
python3 -m venv $path && $path/bin/pip3 install -r requirements.txt
