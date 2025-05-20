idf_component_register(SRCS "comm_layer.c"
                    INCLUDE_DIRS "." "include" 
                    PRIV_REQUIRES mqtt driver json)
