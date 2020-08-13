#!/usr/bin/env bash
IP=$1
DATE="$(sudo date | awk '{print $3 " " $2 " " $6 " " $4}')"
ssh "$IP" "sudo date -s '$DATE';"
