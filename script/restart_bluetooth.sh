#!/bin/sh
sudo rfkill unblock all
sudo systemctl restart bluetooth.service

