#!/usr/bin/env bash
xacro `rospack find mildred_description`/urdf/mildred.urdf.xacro -o `rospack find mildred_description`/urdf/mildred.urdf
check_urdf `rospack find mildred_description`/urdf/mildred.urdf