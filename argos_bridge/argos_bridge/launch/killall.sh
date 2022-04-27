#!/bin/bash

kill $(ps aux | grep '[r]os' | awk '{print $2}')
kill $(ps aux | grep '[a]rgos' | awk '{print $2}')
