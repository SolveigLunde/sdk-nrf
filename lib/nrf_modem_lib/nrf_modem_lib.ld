. = ALIGN(4);
_nrf_modem_lib_dfu_cb_list_start = .;
KEEP(*(SORT_BY_NAME("._nrf_modem_lib_dfu_cb.*")));
_nrf_modem_lib_dfu_cb_list_end = .;
_nrf_modem_lib_init_cb_list_start = .;
KEEP(*(SORT_BY_NAME("._nrf_modem_lib_init_cb.*")));
_nrf_modem_lib_init_cb_list_end = .;
_nrf_modem_lib_shutdown_cb_list_start = .;
KEEP(*(SORT_BY_NAME("._nrf_modem_lib_shutdown_cb.*")));
_nrf_modem_lib_shutdown_cb_list_end = .;
