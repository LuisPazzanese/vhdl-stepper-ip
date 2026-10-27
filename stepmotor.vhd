--------------------------------------------------------------------------------
-- stepmotor.vhd - driver de motor de passo unipolar em half step
--
--   EN / DIR / VEL[1:0]        -> item C
--   numero de passos + START   -> item B
--   rampa de aceleracao        -> item A
--
-- Dois modos de operacao:
--   steps_in = 0  -> CONTINUO: gira enquanto EN estiver ligado, sem START
--   steps_in > 0  -> START executa steps_in * STEPS_MULT meios-passos e para
--
-- Um unico dominio de clock. A velocidade sai de clock enable ('tick'), nunca
-- de clock derivado por contador.
--
-- A teoria (pull-in rate, acumulador de fase, quando frear) esta no README.md.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity stepmotor is
  generic (
    CLK_FREQ   : integer := 50_000_000;  -- 50e6 na placa; use 50_000 no testbench
    STEPS_BITS : integer := 6;
    STEPS_MULT : integer := 64;          -- potencia de 2 -> vira deslocamento
    ACEL       : integer := 3000;        -- meios-passos/s^2 (ajuste de bancada)
    RAMP_HZ    : integer := 1000         -- atualizacoes da rampa por segundo
  );
  port (
    clk        : in  std_logic;
    rst        : in  std_logic;                     -- sincrono, ativo alto
    en         : in  std_logic;
    dir        : in  std_logic;                     -- 1 = horario
    vel        : in  std_logic_vector(1 downto 0);
    steps_in   : in  std_logic_vector(STEPS_BITS-1 downto 0);
    start      : in  std_logic;                     -- borda de subida
    busy       : out std_logic;
    done       : out std_logic;                     -- pulso de 1 ciclo
    estado_out : out std_logic_vector(1 downto 0);  -- 00 parado 01 acel 10 cruz 11 desacel
    steps_left : out std_logic_vector(15 downto 0);
    phases     : out std_logic_vector(3 downto 0)   -- bobinas
  );
end entity stepmotor;

architecture rtl of stepmotor is

  -- Half step: entre estados consecutivos (incluindo o wrap) muda exatamente um
  -- bit, e o numero de bobinas ligadas alterna 1, 2, 1, 2...
  type seq_t is array (0 to 7) of std_logic_vector(3 downto 0);
  constant SEQ : seq_t := (
    "1000", "1100", "0100", "0110", "0010", "0011", "0001", "1001"
  );

  -- Acumulador de fase: taxa = incr * CLK_FREQ / 2^ACC_W, linear em incr.
  constant ACC_W : integer := 26;

  -- TAXA(0) e a velocidade de partida de toda rampa (abaixo do pull-in rate do
  -- 28BYJ-48). TAXA(3) esta acima dele de proposito: so e alcancavel com rampa.
  type taxa_t is array (0 to 3) of integer;
  constant TAXA : taxa_t := (150, 400, 800, 1200);   -- meios-passos/s

  -- Duas divisoes entre constantes: o Quartus resolve em tempo de compilacao.
  -- Nesta ordem para o produto intermediario nao estourar o range de integer.
  function incr_de(taxa : integer) return integer is
  begin
    return (2**ACC_W) / (CLK_FREQ / taxa);
  end function;

  type incr_t is array (0 to 3) of integer;
  constant INCR_TAB : incr_t := (
    incr_de(TAXA(0)), incr_de(TAXA(1)), incr_de(TAXA(2)), incr_de(TAXA(3))
  );
  constant INCR_MIN : integer := INCR_TAB(0);

  constant DINCR     : integer := incr_de(ACEL) / RAMP_HZ;
  constant RAMP_DIV  : integer := CLK_FREQ / RAMP_HZ;
  constant MAX_STEPS : integer := (2**STEPS_BITS - 1) * STEPS_MULT;

  type estado_t is (PARADO, ACELERA, CRUZEIRO, DESACELERA);
  signal estado : estado_t := PARADO;

  -- P1
  signal en_r, dir_r, start_r : std_logic_vector(1 downto 0) := "00";
  signal vel_s1, vel_s2       : std_logic_vector(1 downto 0) := "00";
  signal start_prev, start_pulse : std_logic := '0';

  -- P2
  signal acc  : unsigned(ACC_W-1 downto 0) := (others => '0');
  signal tick : std_logic := '0';

  -- P6
  signal incr      : unsigned(ACC_W-1 downto 0) := to_unsigned(INCR_MIN, ACC_W);
  signal incr_alvo : unsigned(ACC_W-1 downto 0);
  signal ramp_cnt  : integer range 0 to RAMP_DIV-1 := 0;

  signal idx     : integer range 0 to 7         := 0;
  signal restam  : integer range 0 to MAX_STEPS := 0;
  signal n_acel  : integer range 0 to MAX_STEPS := 0;  -- passos gastos acelerando
  signal continuo : std_logic := '0';

  signal movendo : std_logic;

begin

  assert DINCR >= 1
    report "stepmotor: DINCR = 0. Aumente ACEL ou reduza RAMP_HZ." severity failure;
  assert RAMP_DIV >= 1
    report "stepmotor: RAMP_DIV = 0. CLK_FREQ deve ser >= RAMP_HZ." severity failure;

  movendo <= '1' when (estado /= PARADO and en_r(1) = '1') else '0';

  ------------------------------------------------------------------------------
  -- P1 - sincronizador de duas etapas (chaves e botoes sao assincronos) e
  -- deteccao de borda do START. Os KEY sao ativos em baixo; a inversao fica no
  -- toplevel, para este IP trabalhar sempre em logica positiva.
  ------------------------------------------------------------------------------
  p1_sync : process(clk)
  begin
    if rising_edge(clk) then
      en_r    <= en_r(0)    & en;
      dir_r   <= dir_r(0)   & dir;
      start_r <= start_r(0) & start;
      vel_s1  <= vel;
      vel_s2  <= vel_s1;

      start_prev  <= start_r(1);
      start_pulse <= start_r(1) and not start_prev;
    end if;
  end process p1_sync;

  ------------------------------------------------------------------------------
  -- P2 - gerador de tick. Cada transbordo do acumulador e um meio-passo. O
  -- resto nao e descartado, entao o erro de quantizacao nao acumula.
  ------------------------------------------------------------------------------
  p2_tick : process(clk)
    variable soma : unsigned(ACC_W downto 0);
  begin
    if rising_edge(clk) then
      tick <= '0';
      if rst = '1' or movendo = '0' then
        acc <= (others => '0');
      else
        soma := ('0' & acc) + ('0' & incr);
        tick <= soma(ACC_W);
        acc  <= soma(ACC_W-1 downto 0);
      end if;
    end if;
  end process p2_tick;

  ------------------------------------------------------------------------------
  -- P6 - rampa. Degraus iguais de DINCR em intervalos iguais = aceleracao
  -- constante. O mesmo DINCR sobe e desce, o que torna a desaceleracao o
  -- espelho exato da aceleracao - propriedade da qual P5 depende.
  ------------------------------------------------------------------------------
  incr_alvo <= to_unsigned(INCR_MIN, ACC_W)
                 when (estado = DESACELERA or estado = PARADO)
                 else to_unsigned(INCR_TAB(to_integer(unsigned(vel_s2))), ACC_W);

  p6_rampa : process(clk)
  begin
    if rising_edge(clk) then
      if rst = '1' or movendo = '0' then
        incr     <= to_unsigned(INCR_MIN, ACC_W);
        ramp_cnt <= 0;
      elsif ramp_cnt = RAMP_DIV-1 then
        ramp_cnt <= 0;
        if incr + DINCR <= incr_alvo then
          incr <= incr + DINCR;
        elsif incr > incr_alvo + DINCR then
          incr <= incr - DINCR;
        else
          incr <= incr_alvo;          -- encosta no alvo sem oscilar em torno
        end if;
      else
        ramp_cnt <= ramp_cnt + 1;
      end if;
    end if;
  end process p6_rampa;

  ------------------------------------------------------------------------------
  -- P3 - sequenciador. 'tick' e clock enable. idx NAO e zerado ao parar: zerar
  -- daria um salto de ate 7 meios-passos no reinicio.
  ------------------------------------------------------------------------------
  p3_seq : process(clk)
  begin
    if rising_edge(clk) then
      if rst = '1' then
        idx <= 0;
      elsif movendo = '1' and tick = '1' then
        if dir_r(1) = '1' then
          if idx = SEQ'high then idx <= SEQ'low;  else idx <= idx + 1; end if;
        else
          if idx = SEQ'low  then idx <= SEQ'high; else idx <= idx - 1; end if;
        end if;
      end if;
    end if;
  end process p3_seq;

  ------------------------------------------------------------------------------
  -- P5 - controle. Percorre ACELERA -> CRUZEIRO -> DESACELERA.
  --
  -- Quando frear: 'n_acel' conta os passos gastos acelerando; como a descida
  -- espelha a subida, ela cobre a mesma distancia. Logo basta frear quando
  -- restam <= n_acel. Sem multiplicador, sem divisor, e resolve sozinho o
  -- perfil triangular (movimento curto demais para chegar ao cruzeiro).
  --
  -- As comparacoes usam variables com os valores ja atualizados do tick
  -- corrente; usar os sinais antigos custaria um passo de erro na frenagem.
  ------------------------------------------------------------------------------
  p5_ctrl : process(clk)
    variable restam_n : integer range 0 to MAX_STEPS;
    variable n_acel_n : integer range 0 to MAX_STEPS;
  begin
    if rising_edge(clk) then
      done <= '0';

      if rst = '1' or (estado /= PARADO and en_r(1) = '0') then
        estado   <= PARADO;      -- EN desliga o motor a qualquer momento
        restam   <= 0;
        n_acel   <= 0;
        continuo <= '0';

      else
        case estado is

          when PARADO =>
            if en_r(1) = '1' then
              if to_integer(unsigned(steps_in)) = 0 then
                -- Modo continuo: arranca sozinho, sem depender do START.
                continuo <= '1';
                n_acel   <= 0;
                estado   <= ACELERA;
              elsif start_pulse = '1' then
                restam   <= to_integer(unsigned(steps_in)) * STEPS_MULT;
                continuo <= '0';
                n_acel   <= 0;
                estado   <= ACELERA;
              end if;
            end if;

          when ACELERA =>
            if continuo = '1' then
              if incr >= incr_alvo then estado <= CRUZEIRO; end if;
            elsif tick = '1' then
              restam_n := restam - 1;
              n_acel_n := n_acel + 1;
              restam <= restam_n;
              n_acel <= n_acel_n;
              if    restam_n = 0          then estado <= PARADO; done <= '1';
              elsif restam_n <= n_acel_n  then estado <= DESACELERA;
              elsif incr >= incr_alvo     then estado <= CRUZEIRO;
              end if;
            end if;

          when CRUZEIRO =>
            if continuo = '1' then
              if incr < incr_alvo then estado <= ACELERA; end if;
            elsif tick = '1' then
              restam_n := restam - 1;
              restam <= restam_n;
              if    restam_n = 0        then estado <= PARADO; done <= '1';
              elsif restam_n <= n_acel  then estado <= DESACELERA;
              end if;
            elsif incr < incr_alvo then
              estado <= ACELERA;                 -- VEL subiu no meio do caminho
            end if;

          when DESACELERA =>
            if tick = '1' then
              restam_n := restam - 1;
              restam <= restam_n;
              if n_acel > 0 then n_acel <= n_acel - 1; end if;
              if restam_n = 0 then estado <= PARADO; done <= '1'; end if;
            end if;

        end case;
      end if;
    end if;
  end process p5_ctrl;

  ------------------------------------------------------------------------------
  -- P4 - saidas.
  --
  -- EN = 0 -> "0000": bobinas desenergizadas, eixo livre, nao esquenta parado.
  -- EN = 1 parado -> mantem SEQ(idx): torque de retencao, o eixo trava na
  -- posicao final. Quem decide energizar e a chave EN, nao o fim do movimento.
  ------------------------------------------------------------------------------
  phases     <= SEQ(idx) when en_r(1) = '1' else "0000";
  busy       <= '0' when estado = PARADO else '1';
  steps_left <= std_logic_vector(to_unsigned(restam, 16));

  with estado select estado_out <=
    "00" when PARADO,
    "01" when ACELERA,
    "10" when CRUZEIRO,
    "11" when DESACELERA;

end architecture rtl;
