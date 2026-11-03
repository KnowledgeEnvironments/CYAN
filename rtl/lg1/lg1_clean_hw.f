# Synthesizable LG1 hardware-memory profile. Board-specific MIG, HDMI and
# legacy DE2i adapters are deliberately outside this architectural file list.
-F lg1_clean_common.f
lg1_clean_core.v
lg1_write_combiner_64b.v
lg1_line_cache_64b.v
lg1_mem_line_adapter_64b.v
lg1_clean_line_core.v
