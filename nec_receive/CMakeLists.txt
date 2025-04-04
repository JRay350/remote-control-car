
add_library(nec_receive nec_receive.c)

# invoke pio_asm to assemble state machine code
pico_generate_pio_header(nec_receive ${CMAKE_CURRENT_LIST_DIR}/nec_receive.pio)

target_link_libraries(nec_receive PRIVATE
        pico_stdlib
        hardware_pio
        )

target_include_directories (nec_receive PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}
	${CMAKE_CURRENT_BINARY_DIR}
	)
