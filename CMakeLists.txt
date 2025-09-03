# Copyright (c) 2024 The ZMK Contributors
# SPDX-License-Identifier: MIT

if(CONFIG_ZMK_SCROLL_SNAP)
  target_include_directories(app PRIVATE include)

  if ((NOT CONFIG_ZMK_SPLIT) OR CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
    target_sources_ifdef(CONFIG_ZMK_SCROLL_SNAP app PRIVATE
      src/input_processors/input_processor_scroll_snap.c
    )
  endif()
endif()
