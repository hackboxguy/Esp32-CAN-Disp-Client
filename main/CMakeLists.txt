idf_component_register(
    SRCS
        "main.c"
        "can_receiver.c"
        "display_driver.c"
        "display_ui.c"
    INCLUDE_DIRS "."
    REQUIRES driver nvs_flash
)
