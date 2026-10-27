# Joint definitions (radians, metres, source model values)

23 revolute coordinates = 7 arm + 10 independent hand + 6 mimic. Total links 36, joints 35. Effort/velocity are source model fields, not verified hardware limits.

| Joint | Type | Parent → child | Axis | Lower / upper | Closed | Mimic |
|---|---|---|---|---|---|---|
| openarm_body_world_joint | fixed | world → openarm_body_link0 | — | — | fixed | — |
| openarm_right_openarm_body_link0_joint | fixed | openarm_body_link0 → openarm_right_link0 | — | — | fixed | — |
| openarm_right_joint1 | revolute | openarm_right_link0 → openarm_right_link1 | 0 0 1 | -1.396263 / 3.490659 | 0.15 | — |
| openarm_right_joint2 | revolute | openarm_right_link1 → openarm_right_link2 | -1 0 0 | -0.174532673 / 3.31612533 | 0.15 | — |
| openarm_right_joint3 | revolute | openarm_right_link2 → openarm_right_link3 | 0 0 1 | -1.570796 / 1.570796 | -0.15 | — |
| openarm_right_joint4 | revolute | openarm_right_link3 → openarm_right_link4 | 0 1 0 | 0 / 2.443461 | 0.15 | — |
| openarm_right_joint5 | revolute | openarm_right_link4 → openarm_right_link5 | 0 0 1 | -1.570796 / 1.570796 | 0.1 | — |
| openarm_right_joint6 | revolute | openarm_right_link5 → openarm_right_link6 | 1 0 0 | -0.785398 / 0.785398 | -0.1 | — |
| openarm_right_joint7 | revolute | openarm_right_link6 → openarm_right_link7 | 0 1 0 | -1.570796 / 1.570796 | 0.1 | — |
| adapter_mount_joint | fixed | openarm_right_link7 → omnihand_adapter | — | — | fixed | — |
| omni_R_palm_joint | fixed | omni_base_link → omni_R_palm | — | — | fixed | — |
| omni_R_thumb_roll_joint | revolute | omni_R_palm → omni_R_thumb_roll | 1 0 0 | -0.174532925 / 0.872664626 | 0.6 | — |
| omni_R_thumb_abad_joint | revolute | omni_R_thumb_roll → omni_R_thumb_abad | 0 0 1 | -1.74532925 / 0 | -0.5 | — |
| omni_R_thumb_mcp_joint | revolute | omni_R_thumb_abad → omni_R_thumb_mcp | 1 0 0 | 0 / 0.855211333 | 0.45 | — |
| omni_R_thumb_pip_joint | revolute | omni_R_thumb_mcp → omni_R_thumb_pip | 1 0 0 | 0 / 1.13446401 | 0.5985 | {"joint": "omni_R_thumb_mcp_joint", "multiplier": "1.33", "offset": "0"} |
| omni_R_thumb_dip_joint | revolute | omni_R_thumb_pip → omni_R_thumb_dip | 1 0 0 | 0 / 1.13446401 | 0.585 | {"joint": "omni_R_thumb_mcp_joint", "multiplier": "1.3", "offset": "0"} |
| omni_R_thumb_tip_joint | fixed | omni_R_thumb_dip → omni_R_thumb_tip | — | — | fixed | — |
| omni_R_index_abad_joint | revolute | omni_R_palm → omni_R_index_abad | 1 0 0 | -0.20943951 / 0 | 0 | — |
| omni_R_index_pip_joint | revolute | omni_R_index_abad → omni_R_index_pip | 0 1 0 | 0 / 1.57079633 | 0.05 | — |
| omni_R_index_dip_joint | revolute | omni_R_index_pip → omni_R_index_dip | 0 1 0 | 0 / 1.83259571 | 0.05485 | {"joint": "omni_R_index_pip_joint", "multiplier": "1.097", "offset": "0"} |
| omni_R_index_tip_joint | fixed | omni_R_index_dip → omni_R_index_tip | — | — | fixed | — |
| omni_R_middle_abad_joint | fixed | omni_R_palm → omni_R_middle_abad | — | — | fixed | — |
| omni_R_middle_pip_joint | revolute | omni_R_middle_abad → omni_R_middle_pip | 0 1 0 | 0 / 1.57079633 | 0.05 | — |
| omni_R_middle_dip_joint | revolute | omni_R_middle_pip → omni_R_middle_dip | 0 1 0 | 0 / 1.83259571 | 0.05485 | {"joint": "omni_R_middle_pip_joint", "multiplier": "1.097", "offset": "0"} |
| omni_R_middle_tip_joint | fixed | omni_R_middle_dip → omni_R_middle_tip | — | — | fixed | — |
| omni_R_ring_abad_joint | revolute | omni_R_palm → omni_R_ring_abad | 1 0 0 | 0 / 0.174532925 | 0 | — |
| omni_R_ring_pip_joint | revolute | omni_R_ring_abad → omni_R_ring_pip | 0 1 0 | 0 / 1.57079633 | 0.05 | — |
| omni_R_ring_dip_joint | revolute | omni_R_ring_pip → omni_R_ring_dip | 0 1 0 | 0 / 1.83259571 | 0.05485 | {"joint": "omni_R_ring_pip_joint", "multiplier": "1.097", "offset": "0"} |
| omni_R_ring_tip_joint | fixed | omni_R_ring_dip → omni_R_ring_tip | — | — | fixed | — |
| omni_R_pinky_abad_joint | revolute | omni_R_palm → omni_R_pinky_abad | 1 0 0 | 0 / 0.174532925 | 0 | — |
| omni_R_pinky_pip_joint | revolute | omni_R_pinky_abad → omni_R_pinky_pip | 0 1 0 | 0 / 1.57079633 | 0.05 | — |
| omni_R_pinky_dip_joint | revolute | omni_R_pinky_pip → omni_R_pinky_dip | 0 1 0 | 0 / 1.83259571 | 0.05485 | {"joint": "omni_R_pinky_pip_joint", "multiplier": "1.097", "offset": "0"} |
| omni_R_pinky_tip_joint | fixed | omni_R_pinky_dip → omni_R_pinky_tip | — | — | fixed | — |
| omnihand_base_joint | fixed | omnihand_adapter → omni_base_link | — | — | fixed | — |
| tool_tcp_joint | fixed | omni_base_link → tool_tcp | — | — | fixed | — |

## Origins

| Joint | xyz (m) | rpy (rad) |
|---|---|---|
| openarm_body_world_joint | 0 0 0 | 0 0 0 |
| openarm_right_openarm_body_link0_joint | 0.0 -0.031 0.698 | 1.5708 0 0 |
| openarm_right_joint1 | 0.0 0.0 0.0625 | 0 0 0 |
| openarm_right_joint2 | -0.0301 0.0 0.06 | 1.57079632679 0 0 |
| openarm_right_joint3 | 0.0301 0.0 0.06625 | 0 0 0 |
| openarm_right_joint4 | -0.0 0.0315 0.15375 | 0 0 0 |
| openarm_right_joint5 | 0.0 -0.0315 0.0955 | 0 0 0 |
| openarm_right_joint6 | 0.0375 0.0 0.1205 | 0 0 0 |
| openarm_right_joint7 | -0.0375 0.0 0.0 | 0 0 0 |
| adapter_mount_joint | 0 0 0.0495006 | 0 0 0 |
| omni_R_palm_joint | 0 0 0 | 0 0 0 |
| omni_R_thumb_roll_joint | 0 0.018521 0.047551 | 0 0 0 |
| omni_R_thumb_abad_joint | 0.020801 0.004292 -0.00052506 | 0 0 0 |
| omni_R_thumb_mcp_joint | 0 0.0265 -0.005275 | 0 0 0 |
| omni_R_thumb_pip_joint | 0 0.036231 0.0036755 | 0 0 0 |
| omni_R_thumb_dip_joint | 0 0.026468 -0.0073106 | 0 0 0 |
| omni_R_thumb_tip_joint | -1.64798730217797e-17 0.02392 -0.01158 | 0 0 2.77555756156289e-17 |
| omni_R_index_abad_joint | 0 0.02695 0.094548 | 0 0 0 |
| omni_R_index_pip_joint | 0.00048163 0 0.017575 | 0 0 0 |
| omni_R_index_dip_joint |  -0.0015  0 0.036 | 0 0 0 |
| omni_R_index_tip_joint | -0.0011 0 0.03612 | 0 0 0 |
| omni_R_middle_abad_joint | -0.0025 0.00795 0.096548 | 0 0 0 |
| omni_R_middle_pip_joint | 0.00048163 0 0.017575 | 0 0 0 |
| omni_R_middle_dip_joint |  -0.0015  0 0.036 | 0 0 0 |
| omni_R_middle_tip_joint | -0.0011 0 0.03612 | 0 0 0 |
| omni_R_ring_abad_joint | -0.00151 -0.01105 0.094548 | 0 0 0 |
| omni_R_ring_pip_joint | 0.00048163 0 0.017575 | 0 0 0 |
| omni_R_ring_dip_joint |  -0.0015  0 0.036 | 0 0 0 |
| omni_R_ring_tip_joint | -0.0011 0 0.03612 | 0 0 0 |
| omni_R_pinky_abad_joint | 0 -0.03005 0.089548 | 0 0 0 |
| omni_R_pinky_pip_joint | 0.00048163 0 0.017575 | 0 0 0 |
| omni_R_pinky_dip_joint |  -0.0015  0 0.036 | 0 0 0 |
| omni_R_pinky_tip_joint | -0.0011 0 0.03612 | 0 0 0 |
| omnihand_base_joint | 0 0 0.017 | 0 0 3.14159265358979 |
| tool_tcp_joint | 0.0 0.0 0.0 | 0.0 0.0 0.0 |
