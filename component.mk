# Component makefile for GY-NEO6
# include a nmea parser
# hope to god it fits in memory

# expected anyone using this driver includes it as 
INC_DIRS += $(gyneo6_ROOT)..
INC_DIRS += $(gyneo6_ROOT)libnmea/include

gyneo6_INC_DIR =  $(gyneo6_ROOT)libnmea/include/parser
gyneo6_INC_DIR +=  $(gyneo6_ROOT)libnmea/include/nmea

# args for passing into compile rule generation
gyneo6_SRC_DIR += $(gyneo6_ROOT)
gyneo6_SRC_DIR += $(gyneo6_ROOT)libnmea/nmea
gyneo6_SRC_DIR += $(gyneo6_ROOT)libnmea/parsers
$(eval $(call component_compile_rules,gyneo6))