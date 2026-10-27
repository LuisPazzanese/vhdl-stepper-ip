# IP core para motor de passo com curva de aceleração

Driver de motor de passo unipolar (28BYJ-48 + ULN2003) escrito em VHDL para a
DE10-Standard, com controle de direção, quatro velocidades, número programável
de passos e **rampa de aceleração trapezoidal**.

| Item da rubrica | Onde está |
|---|---|
| **C** — EN, DIR, VEL[1:0] | `stepmotor.vhd`, processos `p1_sync`, `p3_seq`, `p4` |
| **B** — número de passos | `stepmotor.vhd`, processo `p5_ctrl` |
| **A** — curva de aceleração | `stepmotor.vhd`, processos `p2_tick`, `p6_rampa` e a FSM de `p5_ctrl` |

### Dois modos de operação

| `SW9:SW4` | Comportamento |
|---|---|
| **zero** | **Contínuo** — gira enquanto `EN` estiver ligado, sem precisar de START |
| **≠ zero** | START executa `SW9:SW4 × 64` meios-passos, com rampa, e para |

O modo contínuo existe para que a demonstração básica dependa só de `EN`, `DIR`
e `VEL` — e também porque "zero passos" como sinônimo de "não faz nada" era uma
armadilha: dava a impressão de projeto quebrado.

---

## 1. O problema: por que um motor de passo precisa de rampa

Um motor de passo é um sistema de **malha aberta**. Ninguém mede onde o eixo
está: o driver simplesmente energiza as bobinas numa sequência e *confia* que o
rotor acompanhou. Isso funciona até certo ponto — e esse ponto tem nome.

- **Pull-in rate**: a maior taxa de passos na qual o motor consegue arrancar a
  partir do repouso. No 28BYJ-48 fica em torno de 200 meios-passos/s.
- **Pull-out rate**: a maior taxa que ele consegue *sustentar* já em movimento.
  É bem mais alta, porque a inércia do rotor passa a ajudar em vez de atrapalhar.

Se você mandar o motor partir direto acima do pull-in rate, o rotor não engata:
ele apenas zune, vibra e não sai do lugar. E se você cortar a velocidade de
supetão no fim do movimento, a inércia leva o eixo além da conta e ele **perde
passos** — que, em malha aberta, nunca mais são recuperados.

A curva de aceleração é o que atravessa essa faixa: parte devagar (abaixo do
pull-in), sobe até a velocidade de cruzeiro (que pode estar bem acima dele) e
freia antes de chegar ao destino.

```
v |        ______________
  |       /              \
  |      /                \
  |_____/                  \_____
  +--------------------------------> t
    ACELERA     CRUZEIRO   DESACELERA
```

Neste projeto a velocidade máxima (1200 meios-passos/s) foi escolhida **de
propósito** acima do pull-in rate. Isso torna o item A verificável a olho nu:
sem a rampa, VEL=`11` faz o motor travar; com a rampa, ele gira.

---

## 2. Como gerar velocidade variável sem divisor: acumulador de fase

A forma ingênua de controlar a velocidade é contar ciclos de clock até um
período alvo. Funciona para velocidades fixas, mas atrapalha na hora de rampar,
por dois motivos:

1. **Período e velocidade são inversamente proporcionais.** Diminuir o período
   em degraus iguais faz a aceleração *crescer* junto com a velocidade — ou
   seja, o motor recebe a aceleração mais agressiva justo na faixa em que tem
   menos torque. É um jeito eficiente de perder passos.
2. Converter uma velocidade desejada em período exige **divisão**, que é cara em
   FPGA.

A solução aqui é um acumulador de fase (a mesma ideia de um DDS/NCO):

```vhdl
soma := ('0' & acc) + ('0' & incr);   -- um bit a mais, para pegar o carry
tick <= soma(ACC_W);                  -- cada transbordo é UM meio-passo
acc  <= soma(ACC_W-1 downto 0);       -- guarda o resto
```

A taxa de passos que sai disso é:

```
taxa = incr * CLK_FREQ / 2^ACC_W          [meios-passos/s]
```

**Linear em `incr`.** Então acelerar é simplesmente somar uma constante em
`incr` a intervalos regulares — e isso dá aceleração *constante*, sem um único
divisor no hardware.

Dois detalhes que importam:

- **O resto não é descartado** entre passos. Se `2^ACC_W` não for múltiplo de
  `incr`, um passo sai um ciclo mais cedo e o próximo compensa. O erro fica
  limitado a um ciclo de clock em vez de acumular ao longo do movimento. (É por
  isso que o testbench tolera ±1 ciclo ao verificar a monotonicidade da rampa.)
- As constantes são calculadas por **divisão entre constantes**, que o Quartus
  resolve em tempo de compilação:

  ```vhdl
  function incr_de(taxa : integer) return integer is
  begin
    return (2**ACC_W) / (CLK_FREQ / taxa);
  end function;
  ```

  Escrito nessa ordem (e não `taxa * 2**ACC_W / CLK_FREQ`) justamente para o
  produto intermediário não estourar o range de 32 bits de `integer`.

Com `ACC_W = 26` a 50 MHz, cada unidade de `incr` vale 0,745 meio-passo/s —
resolução muito além do que o motor distingue.

---

## 3. O ponto sutil: quando começar a frear?

Este é o único lugar do item A onde dá para errar feio.

A resposta de livro-texto é cinemática: a distância de frenagem é `v²/2a`. Mas
isso pede um multiplicador e um divisor, e ainda amarra o código a um formato
específico de curva.

O truque usado aqui é bem mais barato. O processo `p6_rampa` usa **o mesmo
`DINCR` para subir e para descer**, então a desaceleração é o espelho temporal
exato da aceleração — e cobre exatamente a mesma distância. Basta, portanto,
contar quantos passos foram gastos acelerando:

```vhdl
signal n_acel : integer range 0 to MAX_STEPS;  -- só cresce em ACELERA
...
-- frear quando:
restam <= n_acel
```

Três consequências boas, de graça:

- Vale para **qualquer formato de curva**, não só a linear. Se você trocar o
  perfil da rampa (S-curve, exponencial), a regra continua correta.
- Resolve sozinho o **perfil triangular**: num movimento curto demais para
  atingir o cruzeiro, a condição dispara ainda dentro de `ACELERA` e o motor
  freia do meio da rampa. Sem caso especial no código.
- Não custa multiplicador nem divisor — só um contador e um comparador.

Detalhe de implementação que vale o comentário: a comparação usa *variables*
com os valores **já atualizados** do tick corrente (`restam_n`, `n_acel_n`).
Comparar os sinais antigos custaria um passo de erro no ponto de frenagem.

---

## 4. Arquitetura

```
             ┌──────────┐
   chaves ──>│ p1_sync  │  dois flip-flops: mata metaestabilidade
             └────┬─────┘  e transforma START em pulso de 1 ciclo
                  │
        ┌─────────┴──────────┐
        v                    v
   ┌──────────┐        ┌───────────┐
   │ p5_ctrl  │<──────>│ p6_rampa  │  aproxima incr do alvo,
   │   FSM    │ estado └─────┬─────┘  DINCR por vez, a 1 kHz
   └────┬─────┘              │ incr
        │ movendo            v
        │              ┌──────────┐
        └─────────────>│ p2_tick  │  acumulador de fase
                       └────┬─────┘
                            │ tick (clock enable)
                            v
                       ┌──────────┐      ┌──────────┐
                       │  p3_seq  │─────>│    p4    │──> bobinas
                       │   idx    │      │ SEQ(idx) │
                       └──────────┘      └──────────┘
```

Tudo roda num **único domínio de clock** de 50 MHz. Não existe clock derivado:
a velocidade vem de *clock enable* (o sinal `tick`), que é a forma correta de
fazer isso em FPGA — clock gerado por contador não passa pela árvore de clock
dedicada e vira fonte de problemas de timing.

### A tabela de fases (half step)

```vhdl
constant SEQ : seq_t := (
  "1000",  --   0° : bobina 3
  "1100",  --  45° : bobinas 3 + 2
  "0100",  --  90° : bobina 2
  "0110",  -- 135° : bobinas 2 + 1
  "0010",  -- 180° : bobina 1
  "0011",  -- 225° : bobinas 1 + 0
  "0001",  -- 270° : bobina 0
  "1001"   -- 315° : bobinas 0 + 3
);
```

Como conferir se uma tabela de half step está certa: entre estados consecutivos
— **incluindo o wrap** do último para o primeiro — muda exatamente um bit, e o
número de bobinas ligadas alterna 1, 2, 1, 2... (Em full step a regra é outra:
dois bits mudam a cada transição, um subindo e um descendo.)

### Decisão de projeto: as bobinas com o motor parado

- `EN = 0` → `"0000"`: bobinas desenergizadas. O eixo gira livre na mão e o
  motor não esquenta parado. Seguro para bancada.
- `EN = 1` e movimento concluído → mantém `SEQ(idx)`: **torque de retenção**. O
  eixo trava na posição final. De nada adianta contar passos com precisão se o
  eixo escorrega depois de parar.

Ou seja: quem decide energizar é a chave EN, não o fim do movimento.

---

## 5. Mapa de pinos

| Função | Sinal no VHDL | Placa | Pino |
|---|---|---|---|
| Clock 50 MHz | `fpga_clk_50` | — | `PIN_Y26` |
| EN | `stepmotor_en` | SW0 | `PIN_AB30` |
| DIR | `stepmotor_dir` | SW1 | `PIN_Y27` |
| VEL[0] | `stepmotor_vel[0]` | SW2 | `PIN_AB28` |
| VEL[1] | `stepmotor_vel[1]` | SW3 | `PIN_AC30` |
| Passos [5:0] | `stepmotor_steps` | SW9..SW4 | `AA30, AC29, AD30, AC28, V25, W25` |
| START | `stepmotor_start` | KEY0 (ativo baixo) | `PIN_AJ4` |
| RESET | `stepmotor_rst` | KEY3 (ativo baixo) | `PIN_AA15` |
| Bobinas [3:0] | `stepmotor_pio` | GPIO | `AK3, Y16, AK2, W15` |
| LEDs [5:0] | `fpga_led_pio` | LEDR | `AA24, AB23, AC23, AD24, AG25, AF25` |

Número de passos = `SW[9:4] × 64`, ou seja 0 a 4032 meios-passos (504 voltas
elétricas). O multiplicador é potência de dois de propósito: o sintetizador o
implementa como deslocamento, custo zero.

LEDs: os quatro de baixo espelham as bobinas; os dois de cima mostram o estado
da curva — `01` acelerando, `10` cruzeiro, `11` desacelerando. É a forma mais
direta de ver o item A funcionando sem osciloscópio.

---

## 6. Como reproduzir

### Simulação (Questa/ModelSim)

```bash
vlib work && vmap work work
vcom -2008 stepmotor.vhd tb_stepmotor.vhd
vsim -c work.tb_stepmotor -do "run -all; quit -f"
```

O testbench é **auto-verificável** — não depende de olhar waveform. Ele roda o
IP com `CLK_FREQ = 50_000` em vez de 50 MHz: a aritmética das constantes é
idêntica, mas um "segundo" simulado vira 50 000 ciclos, então a rampa inteira
cabe em milissegundos de simulação. Seis casos:

1. Contagem exata de passos e pulso de `done`
2. Perfil trapezoidal: o intervalo entre passos cai, estabiliza e sobe
3. Perfil triangular num movimento curto demais para o cruzeiro
4. `DIR` inverte o sentido da sequência
5. `EN = 0` no meio do movimento aborta e desenergiza
6. `VEL` alterado durante o cruzeiro reabre a rampa
7. Modo contínuo: gira só com `EN`, sem START

Saída esperada: `RESULTADO: TODOS OS CASOS PASSARAM`, com 0 errors e 0 warnings.

### Síntese (Quartus Prime Lite)

```bash
quartus_sh --flow compile Entrega1 -c Entrega_1
```

Resultados de referência (Quartus 25.1std, 5CSXFC6D6F31C6):

| Métrica | Valor |
|---|---|
| Slack de setup (pior caso) | **+13,073 ns** |
| ALMs | 168 / 41 910 (< 1 %) |
| Registradores | 112 |
| Pinos | 23 / 499 |

### Na placa

Grave `Entrega_1.sof` (na **raiz** do projeto, não em `output_files/`) pelo
Programmer do Quartus.

Ligação: pinos 1-4 do header GPIO (`GPIO_D0..D3`) em `IN1..IN4`, pino 11
(`VCC5`) e pino 12 (`GND`) na alimentação da plaquinha ULN2003.

**Modo contínuo** (só depende de `SW0..SW3`):

1. `SW9..SW4` todas para baixo (zero passos)
2. `SW0 = 1` → o motor arranca sozinho, acelera e fica em cruzeiro
3. `SW3:2` muda a velocidade em tempo real; `SW1` inverte o sentido
4. `SW0 = 0` → para e desenergiza as bobinas

**Modo por passos:**

1. `SW9:4 = 001000` (8 × 64 = 512 meios-passos, suficiente para um trapézio completo)
2. Aperte KEY0 → **arranca devagar, acelera por ~0,35 s, cruza e freia** antes
   de parar. Os LEDs 5 e 4 percorrem `01 → 10 → 11 → 00`.

Com `EN = 1` e o motor parado, uma bobina fica energizada (torque de retenção):
**um LED aceso fixo é o comportamento correto**, não defeito.

**Teste de controle**, que é o que prova o item A: aumente `ACEL` no generic
para um valor absurdo (ex.: `1_000_000`, que efetivamente elimina a rampa) e
recompile. Em `VEL = 11` o motor passa a travar e zunir sem girar. Volte para
3000 e ele gira. A curva de aceleração é o que viabiliza a velocidade máxima.

---

## 7. Pontos críticos e o que fazer quando der errado

**O motor não gira, nada acontece.** Confira se as portas do `toplevel.vhd`
têm `set_location_assignment` no `.qsf`. Uma porta sem atribuição é colocada
pelo fitter num pino **qualquer** da FPGA e o projeto compila "com sucesso" sem
acionar nada. Diagnóstico rápido:

```bash
grep -E "^(fpga_clk_50|stepmotor_|fpga_led_pio)" Entrega_1.pin
```

e confira contra a tabela da seção 5. Se você renomear uma porta do toplevel,
atualize o `.qsf` **junto**.

**Slack negativo num projeto de 50 MHz.** Quase sempre é o `.sdc` restringindo
um nome de porta que não existe mais. Quando o `create_clock` não casa com
nada, o Quartus inventa um clock default de altíssima frequência e o relatório
fica cheio de violações fantasmas. Procure por `Info (332101): Design is fully
constrained` no log — se não aparecer, o `.sdc` não está pegando.

**`Error (125048): Error reading Quartus Prime Settings File`.** O parser do
`.qsf` **não aceita comentário `;#` no fim da linha**, apesar de ser sintaxe Tcl
válida. Coloque o comentário numa linha própria começando com `#`.

**Um LED de bobina acende e o motor não se mexe.** É o torque de retenção com
`EN = 1` — está correto. Para girar, ou zere `SW9..SW4` (modo contínuo) ou
coloque um valor não-nulo e aperte KEY0. Com passos = 0 **e** dependendo do
START, o motor ficaria parado para sempre.

**O motor zune, vibra, mas não gira.** Aceleração alta demais para a carga:
reduza o generic `ACEL`. Se acontecer só na partida, `TAXA(0)` está acima do
pull-in rate — reduza também.

**O motor perde posição em movimentos repetidos.** Verifique se
`ACEL` não está agressivo demais na frenagem e se a alimentação aguenta o pico
de corrente. Note que `idx` **não** é zerado ao parar, de propósito: zerar daria
um salto de até 7 meios-passos no reinício.

**Ele gira, mas menos do que o pedido.** Bounce mecânico no KEY0 gerando vários
pulsos de START. Aqui não é problema porque START durante o movimento é
ignorado, mas se você mudar essa regra vai precisar de debounce.

**Cuidado com a placa:** mantenha
`set_global_assignment -name RESERVE_ALL_UNUSED_PINS "AS INPUT TRI-STATED"` no
`.qsf`. Sem isso a FPGA pode sair dirigindo nível lógico em pinos ligados a
periféricos da placa.

---

## 8. Como se aprofundar

- **Algoritmo de Austin** — a referência clássica para rampa de aceleração em
  motor de passo trabalhando com *período* em vez de frequência:
  `c_n = c_{n-1} − 2·c_{n-1}/(4n+1)`. Rampa linear exata, ao custo de um divisor
  por passo. Vale ler para entender o que o acumulador de fase está evitando.
  (D. Austin, *Generate stepper-motor speed profiles in real time*, 2005.)
- **Curva S (jerk limitado)** — em vez de aceleração constante, limite a
  *derivada* da aceleração. Reduz a excitação de ressonâncias mecânicas. No
  código atual dá para implementar rampando o próprio `DINCR` dentro de
  `p6_rampa`. O truque do `n_acel` continua valendo sem mudança alguma.
- **Microstepping** — controlar a *corrente* de cada bobina com PWM senoidal em
  vez de ligar/desligar. Multiplica a resolução angular e reduz drasticamente o
  ruído. Exige driver com controle de corrente (A4988, DRV8825) no lugar do
  ULN2003.
- **Malha fechada** — um encoder no eixo transforma o sistema em malha fechada
  e elimina de vez a perda de passos.
- **Interface com processador** — envolver este IP num *slave* Avalon-MM para
  controlá-lo pelo NIOS II ou pelo HPS, transformando os registradores de VEL,
  DIR e número de passos em endereços de memória. É o caminho natural para as
  próximas entregas da disciplina.
- **Ressonância** — motores de passo têm faixas de velocidade instáveis. Um
  perfil de aceleração bem feito serve também para *atravessar* essas faixas
  rápido em vez de operar dentro delas.

## Referências

- Terasic, *DE10-Standard User Manual* — tabelas de pinagem de SW, KEY, LEDR e GPIO.
- Datasheet do 28BYJ-48 e do ULN2003.
- D. Austin, *Generate stepper-motor speed profiles in real time* (2005).
- Intel/Altera, *Quartus Prime Timing Analyzer User Guide* — SDC e `set_false_path`.
- Projeto de exemplo da disciplina (Embarcados Avançados, Insper), ponto de partida deste IP.
