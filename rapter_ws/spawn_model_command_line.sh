#!/bin/bash

gz service -s /world/baylands/create \
--reqtype gz.msgs.EntityFactory \
--reptype gz.msgs.Boolean \
--timeout 1000 \
--req 'sdf_filename: "/home/root/r2_sim_humble/models/joe_apriltag/model.sdf", name: "joe_apriltag"'

