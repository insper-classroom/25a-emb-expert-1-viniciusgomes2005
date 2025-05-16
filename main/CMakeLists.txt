add_executable(pico_emb
        main.c
)

target_link_libraries(pico_emb pico_stdlib freertos hardware_adc hardware_pwm)
pico_add_extra_outputs(pico_emb)
