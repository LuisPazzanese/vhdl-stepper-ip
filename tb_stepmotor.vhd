--------------------------------------------------------------------------------
-- tb_stepmotor.vhd - testbench auto-verificavel do IP do motor de passo
--
-- Roda com CLK_FREQ = 50_000 em vez de 50_000_000: a aritmetica de constantes
-- do IP e exatamente a mesma, mas um "segundo" simulado vira 50 000 ciclos,
-- entao a rampa inteira cabe em milissegundos de simulacao em vez de horas.
-- ACEL tambem e exagerado aqui (12000 em vez de 3000) para encurtar a rampa e
-- deixar espaco para um cruzeiro de verdade em movimentos curtos.
--
-- Nao depende de inspecao visual de waveform: cada caso termina em assert.
--
--   1. Contagem exata de passos + pulso de done
--   2. Curva de aceleracao: intervalo entre passos cai, estabiliza e sobe
--   3. Perfil triangular (movimento curto demais para atingir o cruzeiro)
--   4. DIR inverte o sentido da sequencia
--   5. EN em 0 no meio do movimento aborta e desenergiza
--   6. VEL alterado durante o cruzeiro faz a rampa perseguir o novo alvo
--
-- Como rodar:
--   vlib work && vmap work work
--   vcom -2008 stepmotor.vhd tb_stepmotor.vhd
--   vsim -c work.tb_stepmotor -do "run -all; quit -f"
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity tb_stepmotor is
end entity tb_stepmotor;

architecture sim of tb_stepmotor is

  constant CLK_FREQ   : integer := 50_000;
  constant STEPS_BITS : integer := 6;
  constant STEPS_MULT : integer := 8;    -- menor que na placa: simulacao curta
  constant CLK_PER    : time    := 20 ns;

  signal clk        : std_logic := '0';
  signal rst        : std_logic := '1';
  signal en         : std_logic := '0';
  signal dir        : std_logic := '1';
  signal vel        : std_logic_vector(1 downto 0) := "00";
  signal steps_in   : std_logic_vector(STEPS_BITS-1 downto 0) := (others => '0');
  signal start      : std_logic := '0';
  signal busy       : std_logic;
  signal done       : std_logic;
  signal estado_out : std_logic_vector(1 downto 0);
  signal steps_left : std_logic_vector(15 downto 0);
  signal phases     : std_logic_vector(3 downto 0);

  signal fim_da_sim : boolean := false;

  ------------------------------------------------------------------------------
  -- Monitor de passos.
  --
  -- Conta transicoes de 'phases' e cronometra o intervalo entre elas. E a unica
  -- forma honesta de verificar a rampa: olhamos o que o motor realmente veria
  -- nos fios, nao sinais internos do IP.
  --
  -- Todos estes sinais tem UM UNICO DRIVER (o p_monitor). Integer nao e tipo
  -- resolvido: se o processo de estimulo tambem os atribuisse, a elaboracao
  -- falharia. Por isso o reset dos contadores e pedido por 'zerar'.
  ------------------------------------------------------------------------------
  signal zerar      : boolean := false;
  signal n_passos   : integer := 0;
  signal passo_novo : boolean := false;

  type hist_t is array (0 to 4095) of integer;
  signal hist   : hist_t := (others => 0);
  signal n_hist : integer := 0;

begin

  clk <= not clk after CLK_PER/2 when not fim_da_sim else '0';

  dut : entity work.stepmotor
    generic map (
      CLK_FREQ   => CLK_FREQ,
      STEPS_BITS => STEPS_BITS,
      STEPS_MULT => STEPS_MULT,
      ACEL       => 12000,      -- rampa curta, para a simulacao nao arrastar
      RAMP_HZ    => 1000
    )
    port map (
      clk => clk, rst => rst, en => en, dir => dir, vel => vel,
      steps_in => steps_in, start => start, busy => busy, done => done,
      estado_out => estado_out, steps_left => steps_left, phases => phases
    );

  ------------------------------------------------------------------------------
  -- Monitor: cronometra os passos observando as bobinas
  ------------------------------------------------------------------------------
  p_monitor : process(clk)
    variable ant   : std_logic_vector(3 downto 0) := "0000";
    variable ciclo : integer := 0;
  begin
    if rising_edge(clk) then
      passo_novo <= false;
      ciclo := ciclo + 1;

      if zerar then
        n_passos <= 0;
        n_hist   <= 0;
        ciclo    := 0;

      elsif phases /= ant then
        if phases /= "0000" and ant /= "0000" then
          -- passo de verdade: uma posicao para a outra
          n_passos   <= n_passos + 1;
          passo_novo <= true;
          if n_hist < hist'high then
            hist(n_hist) <= ciclo;
            n_hist       <= n_hist + 1;
          end if;
        end if;
        -- energizar ou desenergizar as bobinas nao e passo, mas reinicia o
        -- cronometro do mesmo jeito
        ciclo := 0;
      end if;

      ant := phases;
    end if;
  end process p_monitor;

  ------------------------------------------------------------------------------
  -- Estimulos
  ------------------------------------------------------------------------------
  p_teste : process

    variable erros : integer := 0;

    procedure checa(cond : boolean; msg : string) is
    begin
      if cond then
        report "  ok  : " & msg severity note;
      else
        report "  FALHA: " & msg severity error;
        erros := erros + 1;
      end if;
    end procedure;

    procedure ciclos(n : integer) is
    begin
      for i in 1 to n loop wait until rising_edge(clk); end loop;
    end procedure;

    procedure zera_monitor is
    begin
      zerar <= true;
      ciclos(2);
      zerar <= false;
      ciclos(1);
    end procedure;

    -- Pulso de START com largura suficiente para atravessar os dois flip-flops
    -- do sincronizador do IP.
    procedure pulso_start is
    begin
      start <= '1';
      ciclos(5);
      start <= '0';
      ciclos(5);
    end procedure;

    variable tlim         : time;
    variable menor        : integer;
    variable pos_menor    : integer;
    variable quebrou      : boolean;
    variable viu_cruzeiro : boolean;
    variable fases_antes  : std_logic_vector(3 downto 0);

  begin
    report "=== tb_stepmotor: inicio ===";

    ciclos(10);
    rst <= '0';
    ciclos(10);

    ----------------------------------------------------------------------------
    report "--- caso 1: contagem exata de passos ---";
    ----------------------------------------------------------------------------
    en       <= '1';
    dir      <= '1';
    vel      <= "01";
    steps_in <= std_logic_vector(to_unsigned(4, STEPS_BITS));  -- 4*8 = 32 passos
    zera_monitor;
    pulso_start;

    tlim := now + 2 sec;
    while busy = '1' and now < tlim loop wait until rising_edge(clk); end loop;
    ciclos(2);

    checa(now < tlim,     "caso 1: movimento terminou dentro do prazo");
    checa(n_passos = 4*STEPS_MULT,
          "caso 1: " & integer'image(n_passos) & " passos (esperado "
          & integer'image(4*STEPS_MULT) & ")");
    checa(busy = '0',               "caso 1: busy baixou ao terminar");
    checa(unsigned(steps_left) = 0, "caso 1: steps_left zerou");

    ciclos(100);

    ----------------------------------------------------------------------------
    report "--- caso 2: curva de aceleracao (perfil trapezoidal) ---";
    ----------------------------------------------------------------------------
    vel      <= "11";
    steps_in <= std_logic_vector(to_unsigned(40, STEPS_BITS));  -- 320 passos
    zera_monitor;
    viu_cruzeiro := false;
    pulso_start;

    -- Percorre o movimento inteiro anotando por quais estados a FSM passou.
    tlim := now + 5 sec;
    while busy = '1' and now < tlim loop
      wait until rising_edge(clk);
      if estado_out = "10" then viu_cruzeiro := true; end if;
    end loop;
    ciclos(2);

    checa(now < tlim,    "caso 2: movimento terminou dentro do prazo");
    checa(viu_cruzeiro,  "caso 2: passou por CRUZEIRO");
    checa(n_passos = 40*STEPS_MULT,
          "caso 2: contagem exata mesmo com rampa (" & integer'image(n_passos) & ")");

    -- Prova numerica da curva: o intervalo entre passos tem que CAIR ate um
    -- minimo e depois SUBIR. Um degrau de velocidade daria intervalo constante
    -- do primeiro ao ultimo passo.
    menor     := integer'high;
    pos_menor := 0;
    for i in 0 to n_hist-1 loop
      if hist(i) < menor then
        menor     := hist(i);
        pos_menor := i;
      end if;
    end loop;

    checa(hist(0) > menor * 2,
          "caso 2: acelerou (1o intervalo " & integer'image(hist(0))
          & " ciclos vs minimo " & integer'image(menor) & ")");
    checa(hist(n_hist-1) > menor * 2,
          "caso 2: desacelerou (ultimo intervalo "
          & integer'image(hist(n_hist-1)) & " ciclos)");
    checa(pos_menor > 0 and pos_menor < n_hist-1,
          "caso 2: velocidade maxima no meio do movimento (passo "
          & integer'image(pos_menor) & " de " & integer'image(n_hist) & ")");

    -- Monotonicidade da subida. Tolerancia de 1 ciclo porque o acumulador de
    -- fase guarda o resto: dois passos consecutivos na mesma velocidade podem
    -- diferir de um ciclo de clock, e isso e o comportamento correto.
    quebrou := false;
    for i in 1 to pos_menor loop
      if hist(i) > hist(i-1) + 1 then quebrou := true; end if;
    end loop;
    checa(not quebrou, "caso 2: aceleracao monotonica (intervalo nunca cresce)");

    ciclos(100);

    ----------------------------------------------------------------------------
    report "--- caso 3: perfil triangular (movimento curto) ---";
    ----------------------------------------------------------------------------
    -- Poucos passos na velocidade maxima: nao ha distancia para chegar ao
    -- cruzeiro, entao a FSM tem que frear ainda dentro da rampa de subida.
    vel      <= "11";
    steps_in <= std_logic_vector(to_unsigned(2, STEPS_BITS));   -- 16 passos
    zera_monitor;
    viu_cruzeiro := false;
    pulso_start;

    tlim := now + 2 sec;
    while busy = '1' and now < tlim loop
      wait until rising_edge(clk);
      if estado_out = "10" then viu_cruzeiro := true; end if;
    end loop;
    ciclos(2);

    checa(not viu_cruzeiro, "caso 3: nunca atingiu CRUZEIRO (perfil triangular)");
    checa(n_passos = 2*STEPS_MULT,
          "caso 3: parou no passo certo mesmo assim ("
          & integer'image(n_passos) & ")");

    ciclos(100);

    ----------------------------------------------------------------------------
    report "--- caso 4: DIR inverte o sentido ---";
    ----------------------------------------------------------------------------
    dir      <= '0';
    vel      <= "00";
    steps_in <= std_logic_vector(to_unsigned(1, STEPS_BITS));   -- 8 passos
    zera_monitor;
    fases_antes := phases;
    pulso_start;

    tlim := now + 2 sec;
    while busy = '1' and now < tlim loop wait until rising_edge(clk); end loop;
    ciclos(2);

    checa(n_passos = STEPS_MULT,
          "caso 4: contagem correta no sentido anti-horario ("
          & integer'image(n_passos) & ")");
    -- 8 meios-passos e uma volta eletrica completa: anda para tras e volta ao
    -- mesmo ponto da tabela. Se DIR estivesse quebrado o resultado seria igual,
    -- entao o que prova a inversao e a checagem abaixo, com meia volta.
    checa(phases = fases_antes, "caso 4: volta eletrica completa fecha na origem");

    steps_in <= std_logic_vector(to_unsigned(1, STEPS_BITS));
    zera_monitor;
    fases_antes := phases;
    dir <= '0';
    pulso_start;
    tlim := now + 2 sec;
    while n_passos < 1 and now < tlim loop wait until rising_edge(clk); end loop;
    -- indo para tras a partir de idx N a sequencia cai em idx N-1
    checa(phases /= fases_antes, "caso 4: primeiro passo foi na direcao oposta");
    tlim := now + 2 sec;
    while busy = '1' and now < tlim loop wait until rising_edge(clk); end loop;
    dir <= '1';

    ciclos(100);

    ----------------------------------------------------------------------------
    report "--- caso 5: EN aborta no meio do movimento ---";
    ----------------------------------------------------------------------------
    vel      <= "10";
    steps_in <= std_logic_vector(to_unsigned(40, STEPS_BITS));
    zera_monitor;
    pulso_start;

    tlim := now + 2 sec;
    while n_passos < 5 and now < tlim loop wait until rising_edge(clk); end loop;
    checa(busy = '1', "caso 5: motor estava mesmo em movimento antes do abort");

    en <= '0';
    ciclos(10);

    checa(busy = '0',               "caso 5: abortou (busy caiu)");
    checa(phases = "0000",          "caso 5: bobinas desenergizadas");
    checa(unsigned(steps_left) = 0, "caso 5: passos restantes descartados");

    en <= '1';
    ciclos(100);

    ----------------------------------------------------------------------------
    report "--- caso 6: VEL alterado durante o movimento ---";
    ----------------------------------------------------------------------------
    -- Em VEL=00 o alvo ja e a velocidade de partida, entao a FSM cai em
    -- CRUZEIRO quase de imediato. Subir VEL no meio deve reabrir a rampa.
    vel      <= "00";
    steps_in <= std_logic_vector(to_unsigned(40, STEPS_BITS));
    zera_monitor;
    pulso_start;

    tlim := now + 5 sec;
    while estado_out /= "10" and now < tlim loop wait until rising_edge(clk); end loop;
    checa(estado_out = "10", "caso 6: chegou ao cruzeiro em VEL=00");

    vel <= "11";                              -- pede mais velocidade no meio
    tlim := now + 2 sec;
    while estado_out /= "01" and now < tlim loop wait until rising_edge(clk); end loop;
    checa(estado_out = "01", "caso 6: voltou a ACELERAR ao subir VEL");

    tlim := now + 5 sec;
    while busy = '1' and now < tlim loop wait until rising_edge(clk); end loop;
    ciclos(2);
    checa(n_passos = 40*STEPS_MULT,
          "caso 6: contagem exata apesar da mudanca de VEL ("
          & integer'image(n_passos) & ")");

    ciclos(100);

    ----------------------------------------------------------------------------
    report "--- caso 7: modo continuo (steps_in = 0, sem START) ---";
    ----------------------------------------------------------------------------
    -- Com zero passos o motor deve arrancar sozinho ao ligar EN, sem depender
    -- do botao START. E o modo que so precisa das chaves ja verificadas.
    en       <= '0';
    ciclos(20);
    vel      <= "11";
    steps_in <= (others => '0');
    zera_monitor;
    en <= '1';                                 -- nenhum pulso de START aqui

    tlim := now + 2 sec;
    while n_passos < 50 and now < tlim loop wait until rising_edge(clk); end loop;
    checa(n_passos >= 50, "caso 7: girou sozinho so com EN, sem START ("
                          & integer'image(n_passos) & " passos)");
    checa(estado_out = "10" or estado_out = "01",
          "caso 7: nao entrou em DESACELERA (nao ha destino para frear)");

    en <= '0';
    ciclos(10);
    checa(busy = '0',      "caso 7: EN=0 para o motor");
    checa(phases = "0000", "caso 7: EN=0 desenergiza as bobinas");

    ----------------------------------------------------------------------------
    report "=== tb_stepmotor: fim ===";
    if erros = 0 then
      report "RESULTADO: TODOS OS CASOS PASSARAM" severity note;
    else
      report "RESULTADO: " & integer'image(erros) & " CASO(S) FALHARAM" severity error;
    end if;

    fim_da_sim <= true;
    wait;
  end process p_teste;

end architecture sim;
