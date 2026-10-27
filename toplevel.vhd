--------------------------------------------------------------------------------
-- toplevel.vhd  -- DE10-Standard (Cyclone V SoC)
--
-- Liga o driver stepmotor aos perifericos da placa.
--
-- IMPORTANTE - NOMES DAS PORTAS
-- Os nomes abaixo sao EXATAMENTE os que o Entrega_1.qsf ja atribui a pinos
-- (herdados do projeto de exemplo da disciplina). Nao renomeie sem atualizar o
-- .qsf junto: uma porta sem set_location_assignment e colocada pelo fitter em
-- um pino QUALQUER da FPGA, e o motor simplesmente nao recebe nada.
--
-- MAPA DE RECURSOS
--   SW(0)     EN     -> stepmotor_en       PIN_AB30
--   SW(1)     DIR    -> stepmotor_dir      PIN_Y27
--   SW(3..2)  VEL    -> stepmotor_vel      PIN_AC30 / PIN_AB28
--   SW(9..4)  passos -> stepmotor_steps    PIN_W25 .. PIN_AA30
--   KEY(0)    START  -> stepmotor_start    PIN_AJ4   (ativo em BAIXO)
--   KEY(3)    RESET  -> stepmotor_rst      PIN_AA15  (ativo em BAIXO)
--   GPIO      fases  -> stepmotor_pio      PIN_AK3 / Y16 / AK2 / W15
--   LEDs             -> fpga_led_pio       PIN_AA24 .. PIN_AF25
--
-- Numero de passos = SW(9..4) * 64  ->  0 a 4032 meios-passos.
--
-- LEDs: os quatro de baixo espelham as bobinas; os dois de cima mostram o
-- estado da curva de aceleracao (01 acelerando, 10 cruzeiro, 11 desacelerando),
-- que e a maneira mais direta de ver o item A funcionando sem osciloscopio.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity toplevel is
  port (
    fpga_clk_50     : in  std_logic;
    stepmotor_pio   : out std_logic_vector(3 downto 0);
    stepmotor_en    : in  std_logic;
    stepmotor_dir   : in  std_logic;
    stepmotor_vel   : in  std_logic_vector(1 downto 0);
    stepmotor_steps : in  std_logic_vector(5 downto 0);
    stepmotor_start : in  std_logic;                     -- KEY0, ativo em BAIXO
    stepmotor_rst   : in  std_logic;                     -- KEY3, ativo em BAIXO
    fpga_led_pio    : out std_logic_vector(5 downto 0)
  );
end entity toplevel;

architecture rtl of toplevel is

  signal fases  : std_logic_vector(3 downto 0);
  signal estado : std_logic_vector(1 downto 0);
  signal reset  : std_logic;

begin

  -- KEY da DE10-Standard e ativo em BAIXO. Invertemos aqui para que o IP
  -- trabalhe sempre em logica positiva.
  reset <= not stepmotor_rst;

  u1 : entity work.stepmotor
    generic map (
      CLK_FREQ   => 50_000_000,   -- clock real da placa
      STEPS_BITS => 6,            -- SW(9..4)
      STEPS_MULT => 64,           -- potencia de 2 -> vira deslocamento
      ACEL       => 3000,         -- meios-passos/s^2 (ajuste de bancada)
      RAMP_HZ    => 1000
    )
    port map (
      clk        => fpga_clk_50,
      rst        => reset,
      en         => stepmotor_en,
      dir        => stepmotor_dir,
      vel        => stepmotor_vel,
      steps_in   => stepmotor_steps,
      start      => not stepmotor_start,   -- ativo em baixo -> logica positiva
      busy       => open,
      done       => open,
      estado_out => estado,
      steps_left => open,                  -- so usado pelo testbench
      phases     => fases
    );

  ------------------------------------------------------------------------------
  -- Saidas
  ------------------------------------------------------------------------------
  stepmotor_pio <= fases;

  fpga_led_pio(3 downto 0) <= fases;    -- espelho das bobinas, util para depurar
  fpga_led_pio(5 downto 4) <= estado;   -- 01 acelera / 10 cruzeiro / 11 desacelera

end architecture rtl;
