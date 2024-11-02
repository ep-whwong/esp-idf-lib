idf_component_register(
    SRC_DIRS . components/common components/st25dvxxkc NFC07A1
    INCLUDE_DIRS . components/common components/st25dvxxkc NFC07A1
    REQUIRES i2cdev log esp_idf_lib_helpers
)
