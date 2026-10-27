# ============================================================================
# SDC1.sdc - restricoes de tempo
#
# O nome do port TEM que casar com a entity do toplevel. Se nao casar, o
# create_clock nao se aplica a nada e o Quartus inventa um clock default de
# 1 GHz - foi o que aconteceu na versao anterior (slack de -3,875 ns num
# projeto que roda a 50 MHz).
# ============================================================================

# Clock de 50 MHz da placa
create_clock -period 20 [get_ports fpga_clk_50]

# Aplica automaticamente clocks derivados na saida de PLLs (nenhuma aqui)
derive_pll_clocks

# Incerteza (jitter, etc.) do clock, conforme o modelo do dispositivo
derive_clock_uncertainty

# ----------------------------------------------------------------------------
# Caminhos sem requisito de tempo
#
# Chaves e botoes sao assincronos e ja passam pelos sincronizadores de dois
# flip-flops do p1_sync dentro do IP. As bobinas e os LEDs sao lentos: nao ha
# nada do outro lado amostrando com clock.
# ----------------------------------------------------------------------------
set_false_path -from [get_ports {stepmotor_en stepmotor_dir stepmotor_start stepmotor_rst}]
set_false_path -from [get_ports {stepmotor_vel[*] stepmotor_steps[*]}]
set_false_path -to   [get_ports {stepmotor_pio[*] fpga_led_pio[*]}]
