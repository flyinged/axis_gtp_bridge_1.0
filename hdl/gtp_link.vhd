--------------------------------------------------------------------------------
--                       Paul Scherrer Institute (PSI)
--------------------------------------------------------------------------------
-- Unit    : gtp_link.vhd
-- Author  : Goran Marinkovic, Section Diagnostic
-- Version : $Revision: 1.3 $
--------------------------------------------------------------------------------
-- Copyright© PSI, Section Diagnostic
--------------------------------------------------------------------------------
-- Comment : This is the package for the GTPE2 component.
--------------------------------------------------------------------------------
-- Std. library (platform) -----------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;

package gtp_link_package is

   ---------------------------------------------------------------------------
   -- Type
   ---------------------------------------------------------------------------
   type gtp_pll_out_type is record
      -- PLL0 clock Interface
      pll0reset                   : std_logic;
      -- PLL1 clock Interface
      pll1reset                   : std_logic;
   end record gtp_pll_out_type;

   type gtp_pll_in_type is record
      -- PLL0 clock Interface
      pll0lock                    : std_logic;
      pll0clk                     : std_logic;
      pll0refclk                  : std_logic;
      -- PLL1 clock Interface
      pll1lock                    : std_logic;
      pll1clk                     : std_logic;
      pll1refclk                  : std_logic;
   end record gtp_pll_in_type;

   type gtp_ctrl_in_type is record
      -- System Interface
      clk                         : std_logic;
      rst                         : std_logic;
   end record gtp_ctrl_in_type;

   type gtp_rx_in_type is record
      -- Power-Down Interface
      rxpd                        : std_logic_vector( 1 downto  0);
      -- Serial lines
      gtprxp                      : std_logic;
      gtprxn                      : std_logic;
   end record gtp_rx_in_type;

   type gtp_rx_out_type is record
      -- Clock
      rxusrclk                    : std_logic;
      -- Reset
      rxresetdone                 : std_logic;
      -- Data
      rxdata                      : std_logic_vector(15 downto  0);
      -- Status 8B/10B decoder
      rxbyteisaligned             : std_logic; --ML84
      rxbyterealign               : std_logic; --ML84
      rxcommadet                  : std_logic; --ML84
      rxdisperr                   : std_logic_vector( 1 downto  0);
      rxnotintable                : std_logic_vector( 1 downto  0);
      rxcharisk                   : std_logic_vector( 1 downto  0);
   end record gtp_rx_out_type;

   type gtp_tx_in_type is record
      -- Power-Down Interface
      txpd                        : std_logic_vector( 1 downto  0);
      -- Data
      txdata                      : std_logic_vector(15 downto  0);
      -- Status 8B/10B decoder
      txcharisk                   : std_logic_vector( 1 downto  0);
   end record gtp_tx_in_type;

   type gtp_tx_out_type is record
      -- Clock
      txusrclk                    : std_logic;
      -- Reset
      txresetdone                 : std_logic;
      -- Serial lines
      gtptxp                      : std_logic;
      gtptxn                      : std_logic;
   end record gtp_tx_out_type;

   type gtp_in_type is record
      pll                         : gtp_pll_in_type;
      rx                          : gtp_rx_in_type;
      tx                          : gtp_tx_in_type;
   end record gtp_in_type;

   type gtp_quad_in_type is array(0 to 3) of gtp_in_type;

   type gtp_out_type is record
      pll                         : gtp_pll_out_type;
      rx                          : gtp_rx_out_type;
      tx                          : gtp_tx_out_type;
   end record gtp_out_type;

   type gtp_quad_out_type is array(0 to 3) of gtp_out_type;

   -----------------------------------------------------------------------------
   -- Functions
   -----------------------------------------------------------------------------
   function PARAM_PLL_FBDIV_45(REF_CLK_MHz : real; BAUD_RATE_Gbps : real) return integer;
   function PARAM_PLL_FBDIV(REF_CLK_MHz : real; BAUD_RATE_Gbps : real) return integer; 
   function PARAM_PLL_REFCLK_DIV(REF_CLK_MHz : real; BAUD_RATE_Gbps : real) return integer;
   function PARAM_RXTXOUT_DIV(REF_CLK_MHz : real; BAUD_RATE_Gbps : real) return integer;
   function PARAM_USRCLK(BAUD_RATE_Gbps : real) return real;
   
   -----------------------------------------------------------------------------
   -- GTP sync block
   -----------------------------------------------------------------------------
   component gtp_sync_block is
   generic
   (
      INITIALISE                  : bit_vector(5 downto 0) := "000000"
   );
   port
   (
      clk                         : in    std_logic;                          -- Clock to be synced to
      data_in                     : in    std_logic;                          -- Data to be synced
      data_out                    : out   std_logic                           -- Synced data
   );
   end component gtp_sync_block;

   -----------------------------------------------------------------------------
   -- GTP start up
   -----------------------------------------------------------------------------
   component gtp_startup is
   generic
   (
      SIM_RESET_SPEEDUP           : boolean := FALSE;                         -- Set to "TRUE" to speed up simulation
      RX_PLL0_USED                : boolean := FALSE;                         -- Use of QPLL in receive path
      TX_PLL0_USED                : boolean := FALSE;                         -- Use of QPLL in transmit path
      RX_PLL1_USED                : boolean := FALSE;                         -- Use of QPLL in receive path
      TX_PLL1_USED                : boolean := FALSE;                         -- Use of QPLL in transmit path
      CPU_CLK_MHz                 : real := 125.0;                            -- Frequency of the stable clock in [MHz]
      USR_CLK_MHz                 : real := 125.0;                            -- Frequency of the user clock in [MHz]
      BAUD_RATE_Gbps              : real := 5.0                               -- Baudrate in GBit/s
   );
   port
   (
      --------------------------------------------------------------------------
      -- CPU clock side
      --------------------------------------------------------------------------
      -- System
      CPU_CLK                     : in    std_logic;                          -- Stable Clock
      CPU_RST                     : in    std_logic;                          -- User Reset
      -- PLL
      PLL0RESET                   : out   std_logic := '0';                   -- Reset QPLL (only if RX uses QPLL)
      PLL0LOCK                    : in    std_logic;                          -- Lock Detect from the QPLL of the GT
      PLL1RESET                   : out   std_logic := '0';                   -- Reset CPLL (only if RX uses CPLL)
      PLL1LOCK                    : in    std_logic;                          -- Lock Detect from the CPLL of the GT
      -- Resets
      GTRXRESET                   : out   std_logic;
      GTTXRESET                   : out   std_logic;
      -- Status
      RXUSERRDY                   : out   std_logic;
      TXUSERRDY                   : out   std_logic;
      RXRESETDONE_FSM             : out   std_logic;                          -- Reset-sequence has successfully been finished.
      TXRESETDONE_FSM             : out   std_logic;                          -- Reset-sequence has successfully been finished.
      --------------------------------------------------------------------------
      -- DRP clock side
      --------------------------------------------------------------------------
      DRPCLK                      : out   std_logic;
      DRPEN                       : out   std_logic;
      DRPRDY                      : in    std_logic;
      DRPWE                       : out   std_logic;
      DRPADDR                     : out   std_logic_vector( 8 downto  0);
      DRPDI                       : out   std_logic_vector(15 downto  0);
      DRPDO                       : in    std_logic_vector(15 downto  0);
      --------------------------------------------------------------------------
      -- MGT clock side
      --------------------------------------------------------------------------
      -- GTP reset
      RXPMARESETDONE              : in    std_logic;   
      RXRESETDONE                 : in    std_logic;
      TXRESETDONE                 : in    std_logic;
      -- RX delay alignment reset
      RXCDRLOCK                   : in    std_logic;
      RXDLYSRESET                 : out   std_logic;
      RXDLYSRESETDONE             : in    std_logic;
      PHALIGNDONE                 : in    std_logic
   );
   end component gtp_startup;

   -----------------------------------------------------------------------------
   -- GRP PLL
   -----------------------------------------------------------------------------
   component gtp_pll is
   generic
   (
      --------------------------------------------------------------------------
      -- Simulation
      --------------------------------------------------------------------------
      SIM_RESET_SPEEDUP           : string :=  "TRUE";                        -- Set to "true" to speed up sim reset
      SIM_PLL0REFCLK_SEL          : bit_vector := "001";
      SIM_PLL1REFCLK_SEL          : bit_vector := "001";
      --------------------------------------------------------------------------
      -- PLL0
      --------------------------------------------------------------------------
      PLL0_FBDIV                  : integer := 4;
      PLL0_FBDIV_45               : integer := 5;
      PLL0_REFCLK_DIV             : integer := 1;
      --------------------------------------------------------------------------
      -- PLL1
      --------------------------------------------------------------------------
      PLL1_FBDIV                  : integer := 1;
      PLL1_FBDIV_45               : integer := 4;
      PLL1_REFCLK_DIV             : integer := 1
   );
   port
   (
      --------------------------------------------------------------------------
      -- CPU clock side
      --------------------------------------------------------------------------
      -- System
      CPU_CLK                     : in    std_logic;                          -- Stable Clock
      CPU_RST                     : in    std_logic;                          -- User Reset
      --------------------------------------------------------------------------
      -- Reference clock input
      --------------------------------------------------------------------------
      GTREFCLK0                   : in std_logic;
      GTREFCLK1                   : in std_logic;
      --------------------------------------------------------------------------
      -- PLL0
      --------------------------------------------------------------------------
      -- Reset
      PLL0RESET                   : in std_logic;
      -- Ref clock
      PLL0REFCLKSEL               : in std_logic_vector(2 downto 0);
      PLL0REFCLKLOST              : out std_logic;
      -- Output clock
      PLL0LOCK                    : out std_logic;
      PLL0OUTCLK                  : out std_logic;
      PLL0OUTREFCLK               : out std_logic;
      --------------------------------------------------------------------------
      -- PLL1
      --------------------------------------------------------------------------
      -- Reset
      PLL1RESET                   : in std_logic;
      -- Ref clock
      PLL1REFCLKSEL               : in std_logic_vector(2 downto 0);
      PLL1REFCLKLOST              : out std_logic;
      -- Output clock
      PLL1LOCK                    : out std_logic;
      PLL1OUTCLK                  : out std_logic;
      PLL1OUTREFCLK               : out std_logic
   );
   end component gtp_pll;

   -----------------------------------------------------------------------------
   -- GTPE2 wrapper
   -----------------------------------------------------------------------------
   component gtp_link is
   generic
   (
      --------------------------------------------------------------------------
      -- Simulation attributes
      --------------------------------------------------------------------------
      SIM_RESET_SPEEDUP           : boolean := FALSE;                         -- Set to "TRUE" to speed up sim reset
      --------------------------------------------------------------------------
      -- Startup timing
      --------------------------------------------------------------------------
      CPU_CLK_MHz                 : real := 125.0;                            -- Frequency of the stable clock in [MHz]
      REF_CLK_MHz                 : real := 125.0;                            -- Frequency of the reference clock in [MHz]
      BAUD_RATE_Gbps              : real := 5.0                               -- Baudrate in GBit/s
   );
   port
   (
      debug_clk                   : out   std_logic;
      debug                       : out   std_logic_vector(127 downto  0);

      i_gtp_ctrl                  : in    gtp_ctrl_in_type;
      i_gtp                       : in    gtp_in_type;
      o_gtp                       : out   gtp_out_type
   );
   end component gtp_link;

end package gtp_link_package;

package body gtp_link_package is

   -----------------------------------------------------------------------------
   -- Functions
   -----------------------------------------------------------------------------
   function PARAM_PLL_FBDIV_45(REF_CLK_MHz : real; BAUD_RATE_Gbps : real) return integer is
      variable PLL_FBDIV_45       : integer;
   begin
      if (REF_CLK_MHz = 125.0) then
         if    (BAUD_RATE_Gbps = 1.25) then
            PLL_FBDIV_45          := 5;
         elsif (BAUD_RATE_Gbps = 2.5) then
            PLL_FBDIV_45          := 5;
         elsif (BAUD_RATE_Gbps = 3.125) then
            PLL_FBDIV_45          := 5;
         elsif (BAUD_RATE_Gbps = 5.0) then
            PLL_FBDIV_45          := 5;
         else
            PLL_FBDIV_45          := 5;
            assert false report "Invalid GTP baud rate = " & real'image(BAUD_RATE_Gbps) severity error;
         end if;
      else
         PLL_FBDIV_45             := 5;
         assert false report "Invalid GTP reference frequency = " & real'image(REF_CLK_MHz) severity error;
      end if;
      return PLL_FBDIV_45  ;
   end PARAM_PLL_FBDIV_45;

   function PARAM_PLL_FBDIV(REF_CLK_MHz : real; BAUD_RATE_Gbps : real) return integer is
      variable PLL_FBDIV          : integer;
   begin
      if (REF_CLK_MHz = 125.0) then
         if    (BAUD_RATE_Gbps = 1.25) then
            PLL_FBDIV             := 4;
         elsif (BAUD_RATE_Gbps = 2.5) then
            PLL_FBDIV             := 4;
         elsif (BAUD_RATE_Gbps = 3.125) then
            PLL_FBDIV             := 5;
         elsif (BAUD_RATE_Gbps = 5.0) then
            PLL_FBDIV             := 4;
         else
            PLL_FBDIV             := 4;
            assert false report "Invalid GTP baud rate = " & real'image(BAUD_RATE_Gbps) severity error;
         end if;
      else
         PLL_FBDIV                := 4;
         assert false report "Invalid GTP reference frequency = " & real'image(REF_CLK_MHz) severity error;
      end if;
      return PLL_FBDIV ;
   end PARAM_PLL_FBDIV;

   function PARAM_PLL_REFCLK_DIV(REF_CLK_MHz : real; BAUD_RATE_Gbps : real) return integer is
      variable PLL_REFCLK_DIV     : integer;
   begin
      if (REF_CLK_MHz = 125.0) then
         if    (BAUD_RATE_Gbps = 1.25) then
            PLL_REFCLK_DIV        := 1;
         elsif (BAUD_RATE_Gbps = 2.5) then
            PLL_REFCLK_DIV        := 1;
         elsif (BAUD_RATE_Gbps = 3.125) then
            PLL_REFCLK_DIV        := 1;
         elsif (BAUD_RATE_Gbps = 5.0) then
            PLL_REFCLK_DIV        := 1;
         else
            PLL_REFCLK_DIV        := 1;
            assert false report "Invalid GTP baud rate = " & real'image(BAUD_RATE_Gbps) severity error;
         end if;
      else
         PLL_REFCLK_DIV           := 1;
         assert false report "Invalid GTP reference frequency = " & real'image(REF_CLK_MHz) severity error;
      end if;
      return PLL_REFCLK_DIV ;
   end PARAM_PLL_REFCLK_DIV;

   function PARAM_RXTXOUT_DIV(REF_CLK_MHz : real; BAUD_RATE_Gbps : real) return integer is
      variable RXTXOUT_DIV        : integer;
   begin
      if (REF_CLK_MHz = 125.0) then
         if    (BAUD_RATE_Gbps = 1.25) then
            RXTXOUT_DIV           := 4;
         elsif (BAUD_RATE_Gbps = 2.5) then
            RXTXOUT_DIV           := 2;
         elsif (BAUD_RATE_Gbps = 3.125) then
            RXTXOUT_DIV           := 2;
         elsif (BAUD_RATE_Gbps = 5.0) then
            RXTXOUT_DIV           := 1;
         else
            RXTXOUT_DIV           := 4;
            assert false report "Invalid GTP baud rate = " & real'image(BAUD_RATE_Gbps) severity error;
         end if;
      else
         RXTXOUT_DIV              := 4;
         assert false report "Invalid GTP reference frequency = " & real'image(REF_CLK_MHz) severity error;
      end if;
      return RXTXOUT_DIV;
   end PARAM_RXTXOUT_DIV;

   function PARAM_USRCLK(BAUD_RATE_Gbps : real) return real is
   begin
      return (BAUD_RATE_Gbps * 1000.0 / 20.0);
   end PARAM_USRCLK;

end package body gtp_link_package;

--------------------------------------------------------------------------------
-- GTP sync block
--------------------------------------------------------------------------------
-- Std. library (platform) -----------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- Work library (platform) -----------------------------------------------------
library unisim;
use unisim.vcomponents.all;

entity gtp_sync_block is
   generic
   (
      INITIALISE                  : bit_vector(5 downto 0) := "000000"
   );
   port
   (
      clk                         : in    std_logic;                          -- Clock to be synced to
      data_in                     : in    std_logic;                          -- Data to be synced
      data_out                    : out   std_logic                           -- Synced data
   );
end gtp_sync_block;

architecture structural of gtp_sync_block is

   -- Internal Signals
   signal   data_sync             : std_logic_vector( 5 downto  0);

begin

   data_sync( 0)                  <= data_in;

   generate_loop: for i in 0 to 4 generate

      -- These attributes will stop timing errors being reported in back annotated
      -- SDF simulation.
      attribute ASYNC_REG                             : string;
      attribute ASYNC_REG of data_sync_fd_inst        : label is "true";
      -- These attributes will stop XST translating the desired flip-flops into an
      -- SRL based shift register.
      attribute shreg_extract                         : string;
      attribute shreg_extract of data_sync_fd_inst    : label is "no";

   begin

      data_sync_fd_inst: FD
      generic map
      (
         INIT                     => INITIALISE( i)
      )
      port map
      (
         C                        => clk,
         D                        => data_sync( i),
         Q                        => data_sync( i+1)
      );

   end generate;

   data_out                       <= data_sync( 5);

end structural;

--------------------------------------------------------------------------------
-- GTP start up
--------------------------------------------------------------------------------
-- Std. library (platform) -----------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- Work library (application) --------------------------------------------------
use work.gtp_link_package.all;

entity gtp_startup is
   generic
   (
      SIM_RESET_SPEEDUP           : boolean := FALSE;                         -- Set to "TRUE" to speed up simulation
      RX_PLL0_USED                : boolean := FALSE;                         -- Use of QPLL in receive path
      TX_PLL0_USED                : boolean := FALSE;                         -- Use of QPLL in transmit path
      RX_PLL1_USED                : boolean := FALSE;                         -- Use of QPLL in receive path
      TX_PLL1_USED                : boolean := FALSE;                         -- Use of QPLL in transmit path
      CPU_CLK_MHz                 : real := 125.0;                            -- Frequency of the stable clock in [MHz]
      USR_CLK_MHz                 : real := 125.0;                            -- Frequency of the user clock in [MHz]
      BAUD_RATE_Gbps              : real := 5.0                               -- Baudrate in GBit/s
   );
   port
   (
      --------------------------------------------------------------------------
      -- CPU clock side
      --------------------------------------------------------------------------
      -- System
      CPU_CLK                     : in    std_logic;                          -- Stable Clock
      CPU_RST                     : in    std_logic;                          -- User Reset
      -- PLL
      PLL0RESET                   : out   std_logic := '0';                   -- Reset QPLL (only if RX uses QPLL)
      PLL0LOCK                    : in    std_logic;                          -- Lock Detect from the QPLL of the GT
      PLL1RESET                   : out   std_logic := '0';                   -- Reset CPLL (only if RX uses CPLL)
      PLL1LOCK                    : in    std_logic;                          -- Lock Detect from the CPLL of the GT
      -- Resets
      GTRXRESET                   : out   std_logic;
      GTTXRESET                   : out   std_logic;
      -- Status
      RXUSERRDY                   : out   std_logic;
      TXUSERRDY                   : out   std_logic;
      RXRESETDONE_FSM             : out   std_logic;                          -- Reset-sequence has successfully been finished.
      TXRESETDONE_FSM             : out   std_logic;                          -- Reset-sequence has successfully been finished.
      --------------------------------------------------------------------------
      -- DRP clock side
      --------------------------------------------------------------------------
      DRPCLK                      : out   std_logic;
      DRPEN                       : out   std_logic;
      DRPRDY                      : in    std_logic;
      DRPWE                       : out   std_logic;
      DRPADDR                     : out   std_logic_vector( 8 downto  0);
      DRPDI                       : out   std_logic_vector(15 downto  0);
      DRPDO                       : in    std_logic_vector(15 downto  0);
      --------------------------------------------------------------------------
      -- MGT clock side
      --------------------------------------------------------------------------
      -- GTP reset
      RXPMARESETDONE              : in    std_logic;   
      RXRESETDONE                 : in    std_logic;
      TXRESETDONE                 : in    std_logic;
      -- RX delay alignment reset
      RXCDRLOCK                   : in    std_logic;
      RXDLYSRESET                 : out   std_logic;
      RXDLYSRESETDONE             : in    std_logic;
      PHALIGNDONE                 : in    std_logic
   );
end gtp_startup;

architecture behavioral of gtp_startup is

   type state_type is
   (
      CONFIG,
      ASSERT_RESET,
      PLL_WAIT,
      PLL_LOCK,
      DRP_ADDR_X11_RD_ORIG,
      DRP_ADDR_X11_RD_ORIG_WAIT,
      DRP_ADDR_X11_WR_CORR,
      DRP_ADDR_X11_WR_CORR_WAIT,
      PMARESET_WAIT,
      DRP_ADDR_X11_WR_ORIG,
      DRP_ADDR_X11_WR_ORIG_DONE,
      RECCLK_STABLE,
      GT_RESET_DONE,
      PHALIGNMENT_INIT,
      PHALIGNMENT_RESET_DONE,
      PHALIGNMENT_CNT_DONE,
      DONE
   );

   signal   state                 : state_type := CONFIG;

   constant CFG_WAIT_TIME         : real := 510.0;                         -- AR43482: Transceiver needs to wait for at least 500 ns after CONFIG (+10 ns addon)
   constant CFG_WAIT_CYCLES       : integer := integer(CFG_WAIT_TIME / 1000.0 * CPU_CLK_MHz) + 100;  -- Number of Clock-Cycles to wait after CONFIG including PLLs are powered down for 100 clock cycles after configuration
   signal   cfg_wait_timer        : integer range 0 to CFG_WAIT_CYCLES := CFG_WAIT_CYCLES;

   constant PLLRESET_CYCLES       : integer := integer(2000.0 * CPU_CLK_MHz); -- 2 ms time out
   signal   pllreset_timer        : integer range 0 to PLLRESET_CYCLES := PLLRESET_CYCLES;
   signal   pllreset              : std_logic_vector( 3 downto  0) := "0000";

   constant PLLRESET_LOCK_CYCLES  : integer := integer(1000.0 * CPU_CLK_MHz); -- 1 ms PLL lock initial time                           -- PLL Lock Time
   signal   pllreset_lock_timer   : integer range 0 to PLLRESET_LOCK_CYCLES := PLLRESET_LOCK_CYCLES;

   signal   drp_data              : std_logic_vector(15 downto  0) := X"0000";

   signal   rxpmaresetdone_sync   : std_logic := '0';
   signal   rxpmaresetdone_sync_d : std_logic_vector( 1 downto  0) := "00";

   signal   gtrxreset_fsm         : std_logic := '0';
   signal   rxresetdone_sync      : std_logic := '0';
   signal   gttxreset_fsm         : std_logic := '0';
   signal   txresetdone_sync      : std_logic := '0';

   function get_cdrlock_time(SIM_RESET_SPEEDUP: in boolean) return real is
      variable lock_time          : real;
   begin
      if (SIM_RESET_SPEEDUP = TRUE) then
         lock_time                := 100.0;
      else
         lock_time                := 50000.0 / BAUD_RATE_Gbps;                      --Typical CDR lock time is 50,000UI as per DS181
      end if;
      return lock_time;
   end function;

   constant RX_CDRLOCK_TIME       : real := get_cdrlock_time(SIM_RESET_SPEEDUP);
   constant RX_CDRLOCK_CYCLES     : integer := integer(RX_CDRLOCK_TIME / 1000.0 * CPU_CLK_MHz);
   signal   rx_cdrlock_cnt        : integer range 0 to RX_CDRLOCK_CYCLES := RX_CDRLOCK_CYCLES;

   signal   userrdy               : std_logic := '0';

   constant RXDLYSRESET_CYCLES    : integer := integer(30.0 / 1000.0 * CPU_CLK_MHz); -- The datasheet states duration RXDLYSRESET < 50 ns
   signal   rxdlysreset_cnt       : integer range 0 to RXDLYSRESET_CYCLES := 0;

   constant PHALIGNMENT_CYCLES    : integer := integer(5000.0 * CPU_CLK_MHz / USR_CLK_MHz); --5000 RXUSRCLK cycles is the max time for Multi lanes designs
   signal   phalignment_timer     : integer range 0 to PHALIGNMENT_CYCLES := PHALIGNMENT_CYCLES;

   signal   rxdlysresetdone_sync  : std_logic := '0';

   signal   rxcdrlock_sync        : std_logic := '0';
   signal   rxcdrlock_timeout     : integer range 0 to 100 := 100;
   
   signal   phaligndone_sync      : std_logic := '0';
   signal   phaligndone_sync_d    : std_logic := '0';
   signal   phaligndone_cnt       : std_logic_vector( 1 downto  0) := "00";

begin

   -----------------------------------------------------------------------------
   -- FSM
   -----------------------------------------------------------------------------
   state_proc: process(CPU_CLK)
   begin
      if rising_edge(CPU_CLK) then
         if (CPU_RST = '1' ) then
            state                 <= CONFIG;
         elsif ((pllreset_timer    = 0) or
                (phalignment_timer = 0)) then
            state                 <= ASSERT_RESET;
         else
            case state is
            -- Wait after configuration
            when CONFIG =>
               if (cfg_wait_timer = 0) then
                 state            <= ASSERT_RESET;
               end if;

            -- Assert resets
            when ASSERT_RESET =>
               state              <= PLL_WAIT;

            -- PLL reset
            when  PLL_WAIT =>
               if (pllreset_lock_timer = 0) then
                  state           <=  PLL_LOCK;
               end if;
            when PLL_LOCK =>
               if (((RX_PLL0_USED and (PLL0LOCK = '1')) or
                    (RX_PLL1_USED and (PLL1LOCK = '1'))) and
                   ((TX_PLL0_USED and (PLL0LOCK = '1')) or
                    (TX_PLL1_USED and (PLL1LOCK = '1')))) then
                 state            <= DRP_ADDR_X11_RD_ORIG;
               end if;

            -- DRP settin of RX_DATA_WIDTH
            when DRP_ADDR_X11_RD_ORIG =>
               state              <= DRP_ADDR_X11_RD_ORIG_WAIT;
            when DRP_ADDR_X11_RD_ORIG_WAIT =>
               if (DRPRDY = '1') then
                  state           <= DRP_ADDR_X11_WR_CORR;
               end if;
            when DRP_ADDR_X11_WR_CORR =>
               state              <= DRP_ADDR_X11_WR_CORR_WAIT;
            when DRP_ADDR_X11_WR_CORR_WAIT =>
               if (DRPRDY = '1') then
                  state <= PMARESET_WAIT;
               end if;
            when PMARESET_WAIT =>
               if (rxpmaresetdone_sync_d = "01") then
                  state           <= DRP_ADDR_X11_WR_ORIG;
               end if;
            when DRP_ADDR_X11_WR_ORIG =>
               state               <= DRP_ADDR_X11_WR_ORIG_DONE;
            when DRP_ADDR_X11_WR_ORIG_DONE =>
               if (DRPRDY = '1') then
                  state            <= RECCLK_STABLE;
               end if;

            -- Wait recovered clock
            when RECCLK_STABLE =>
               if (rx_cdrlock_cnt = 0) then
                  state           <= GT_RESET_DONE;
               end if;

            -- GTP reset is done
            when GT_RESET_DONE =>
               if ((rxresetdone_sync = '1') and
                   (txresetdone_sync = '1')) then
                  state           <= PHALIGNMENT_INIT;
               end if;

            -- Phase alinment
            when PHALIGNMENT_INIT =>
               state              <= PHALIGNMENT_RESET_DONE;
            when PHALIGNMENT_RESET_DONE =>
               if (rxdlysresetdone_sync = '1') then
                  state           <= PHALIGNMENT_CNT_DONE;
               end if;
            when PHALIGNMENT_CNT_DONE =>
               if (phaligndone_cnt( 1) = '1') then
                  state           <= DONE;
               end if;

            -- Phase alinment
            when DONE =>
               if (rxcdrlock_timeout = 0) then
                  state           <= DRP_ADDR_X11_RD_ORIG;
               end if;

            -- GTP reset is done
            when others =>
               state              <= CONFIG;
            end case;
         end if;
      end if;
   end process state_proc;

   -----------------------------------------------------------------------------
   -- AR43482: Transceiver needs to wait for 500 ns after CONFIG + the power down time of the PLLs
   -----------------------------------------------------------------------------
   process(CPU_CLK)
   begin
      if rising_edge(CPU_CLK) then
         if (state = CONFIG) then
            cfg_wait_timer        <= cfg_wait_timer - 1;
         else
            cfg_wait_timer        <= CFG_WAIT_CYCLES;
         end if;
      end if;
   end process;

   -----------------------------------------------------------------------------
   -- Time out
   -----------------------------------------------------------------------------
   process(CPU_CLK)
   begin
      if rising_edge(CPU_CLK) then
         if ((state = PLL_LOCK                 ) or
             (state = DRP_ADDR_X11_RD_ORIG_WAIT) or
             (state = DRP_ADDR_X11_WR_CORR_WAIT) or
             (state = PMARESET_WAIT            ) or
             (state = DRP_ADDR_X11_WR_ORIG_DONE) or
             (state = GT_RESET_DONE            )) then
            pllreset_timer        <= pllreset_timer - 1;
         else
            pllreset_timer        <= PLLRESET_CYCLES;
         end if;
      end if;
   end process;

   -----------------------------------------------------------------------------
   -- Assert PLL reset for min. 1 reference clock period
   -----------------------------------------------------------------------------
   process(CPU_CLK)
   begin
      if rising_edge(CPU_CLK) then
         if (state = ASSERT_RESET) then
            pllreset              <= "1111";
         else
            pllreset              <= pllreset( 2 downto  0) & '0';
         end if;
      end if;
   end process;

   PLL0RESET                      <= pllreset( 3) when ((RX_PLL0_USED = TRUE) or (TX_PLL0_USED = TRUE)) else '0';
   PLL1RESET                      <= pllreset( 3) when ((RX_PLL1_USED = TRUE) or (TX_PLL1_USED = TRUE)) else '0';

   -----------------------------------------------------------------------------
   -- Wait for PLL lock time or QPLL lock time after reset
   -----------------------------------------------------------------------------
   process(CPU_CLK)
   begin
      if rising_edge(CPU_CLK) then
         if (pllreset( 3) = '1') then
            pllreset_lock_timer   <= PLLRESET_LOCK_CYCLES;
         elsif (pllreset_lock_timer > 0) then
            pllreset_lock_timer   <= pllreset_lock_timer - 1;
         end if;
      end if;
   end process;

   -----------------------------------------------------------------------------
   -- DRP settings
   -----------------------------------------------------------------------------
   DRPCLK                         <= CPU_CLK;

   process(CPU_CLK)
   begin
      if rising_edge(CPU_CLK) then
         case state is
         when DRP_ADDR_X11_RD_ORIG =>
            DRPADDR               <= "000010001";
            DRPEN                 <= '1';
            DRPWE                 <= '0';
            DRPDI                 <= X"0000";
         when DRP_ADDR_X11_RD_ORIG_WAIT =>
            DRPADDR               <= "000000000";
            DRPEN                 <= '0';
            DRPWE                 <= '0';
            DRPDI                 <= X"0000";
            if (DRPRDY = '1') then
               drp_data           <= DRPDO;
            end if;
         when DRP_ADDR_X11_WR_CORR =>
            DRPADDR               <= "000010001";
            DRPEN                 <= '1';
            DRPWE                 <= '1';
            DRPDI                 <= drp_data(15 downto 14) & "100" & drp_data(10 downto 0); -- RX_DATA_WIDTH to 32 bit
         when DRP_ADDR_X11_WR_ORIG =>
            DRPADDR               <= "000010001";
            DRPEN                 <= '1';
            DRPWE                 <= '1';
            DRPDI                 <= drp_data(15 downto  0); --restore user setting per prev read
         when others =>
            DRPADDR               <= "000000000";
            DRPEN                 <= '0';
            DRPWE                 <= '0';
            DRPDI                 <= X"0000";
         end case;
      end if;
   end process;

   -----------------------------------------------------------------------------
   -- GTP rx reset
   -----------------------------------------------------------------------------
   process(CPU_CLK)
   begin
      if rising_edge(CPU_CLK) then
         if ((state = ASSERT_RESET             ) or
             (state = PLL_WAIT                 ) or
             (state = PLL_LOCK                 ) or
             (state = DRP_ADDR_X11_RD_ORIG     ) or
             (state = DRP_ADDR_X11_RD_ORIG_WAIT) or
             (state = DRP_ADDR_X11_WR_CORR     ) or
             (state = DRP_ADDR_X11_WR_CORR_WAIT)) then
            gtrxreset_fsm         <= '1';
         else
            gtrxreset_fsm         <= '0';
         end if;
      end if;
   end process;

   GTRXRESET                      <= gtrxreset_fsm;

   -----------------------------------------------------------------------------
   -- GTP tx reset
   -----------------------------------------------------------------------------
   process(CPU_CLK)
   begin
      if rising_edge(CPU_CLK) then
         if ((state = ASSERT_RESET) or
             (state = PLL_WAIT    ) or
             (state = PLL_LOCK    )) then
            gttxreset_fsm         <= '1';
         else
            gttxreset_fsm         <= '0';
         end if;
      end if;
   end process;

   GTTXRESET                      <= gttxreset_fsm;

   -----------------------------------------------------------------------------
   -- User ready
   -----------------------------------------------------------------------------
   process(CPU_CLK)
   begin
      if rising_edge(CPU_CLK) then
         if ((state = GT_RESET_DONE         ) or
             (state = PHALIGNMENT_INIT      ) or
             (state = PHALIGNMENT_RESET_DONE) or
             (state = PHALIGNMENT_CNT_DONE  ) or
             (state = DONE                  )) then
            userrdy               <= '1';
         else
            userrdy               <= '0';
         end if;
      end if;
   end process;

   RXUSERRDY                      <= userrdy;
   TXUSERRDY                      <= userrdy;

   -----------------------------------------------------------------------------
   -- CDR locked status
   -----------------------------------------------------------------------------
   process(CPU_CLK)
   begin
      if rising_edge(CPU_CLK) then
         if (gtrxreset_fsm = '1') then
            rx_cdrlock_cnt        <= RX_CDRLOCK_CYCLES;
         elsif (rx_cdrlock_cnt /= 0) then
            rx_cdrlock_cnt        <= rx_cdrlock_cnt - 1;
         end if;
      end if;
   end process;

   -----------------------------------------------------------------------------
   -- Clock Domain Crossing reset done signal
   -----------------------------------------------------------------------------
   rxpmaresetdone_gtp_sync_block_inst: gtp_sync_block
   port map
   (
      clk                         => CPU_CLK,
      data_in                     => RXPMARESETDONE,
      data_out                    => rxpmaresetdone_sync
   );

   rxresetdone_gtp_sync_block_inst: gtp_sync_block
   port map
   (
      clk                         => CPU_CLK,
      data_in                     => RXRESETDONE,
      data_out                    => rxresetdone_sync
   );

   txresetdone_gtp_sync_block_inst: gtp_sync_block
   port map
   (
      clk                         => CPU_CLK,
      data_in                     => TXRESETDONE,
      data_out                    => txresetdone_sync
   );

   -----------------------------------------------------------------------------
   -- RXPMARESETDONE edge detector
   -----------------------------------------------------------------------------
   process(CPU_CLK)
   begin
      if rising_edge(CPU_CLK) then
         rxpmaresetdone_sync_d    <= rxpmaresetdone_sync_d( 0) & rxpmaresetdone_sync;
      end if;
   end process;

   -----------------------------------------------------------------------------
   -- Phase alignment timeout
   -----------------------------------------------------------------------------
   process(CPU_CLK)
   begin
      if rising_edge(CPU_CLK) then
         if ((state = PHALIGNMENT_RESET_DONE) or
             (state = PHALIGNMENT_CNT_DONE  )) then
            phalignment_timer     <= phalignment_timer - 1;
         else
            phalignment_timer     <= PHALIGNMENT_CYCLES;
         end if;
      end if;
   end process;

   -----------------------------------------------------------------------------
   -- Phase alignment DLYRESET pulse
   -----------------------------------------------------------------------------
   process(CPU_CLK)
   begin
      if rising_edge(CPU_CLK) then
         if (state = PHALIGNMENT_INIT) then
            rxdlysreset_cnt       <= RXDLYSRESET_CYCLES;
         else
            if (rxdlysreset_cnt /= 0) then
               rxdlysreset_cnt    <= rxdlysreset_cnt - 1;
            end if;
         end if;
      end if;
   end process;

   RXDLYSRESET                    <= '0' when (rxdlysreset_cnt = 0) else '1';

   -----------------------------------------------------------------------------
   -- Phase alignment DLYRESET done
   -----------------------------------------------------------------------------
   gtp_sync_block_dlysresetdone: gtp_sync_block
   port map
   (
      clk                         => CPU_CLK,
      data_in                     => RXDLYSRESETDONE,
      data_out                    => rxdlysresetdone_sync
   );

   -----------------------------------------------------------------------------
   -- Phase alignment DLYRESET done
   -----------------------------------------------------------------------------
   gtp_sync_block_rxcdrlock: gtp_sync_block
   port map
   (
      clk                         => CPU_CLK,
      data_in                     => RXCDRLOCK,
      data_out                    => rxcdrlock_sync
   );

   process(CPU_CLK)
   begin
      if rising_edge(CPU_CLK) then
         if (rxcdrlock_sync = '1') then
            rxcdrlock_timeout     <= 100;
         else
            if (rxcdrlock_timeout /= 0) then
               rxcdrlock_timeout  <= rxcdrlock_timeout - 1;
            end if;
         end if;
      end if;
   end process;
   
   -----------------------------------------------------------------------------
   -- Phase alignment data
   -----------------------------------------------------------------------------
   gtp_sync_block_phaligndone: gtp_sync_block
   port map
   (
      clk                         =>  CPU_CLK,
      data_in                     =>  PHALIGNDONE,
      data_out                    =>  phaligndone_sync
   );

   process(CPU_CLK)
   begin
      if rising_edge(CPU_CLK) then
         phaligndone_sync_d       <= phaligndone_sync;
      end if;
   end process;

   process(CPU_CLK)
   begin
      if rising_edge(CPU_CLK) then
         if (state = PHALIGNMENT_CNT_DONE) then
            if (phaligndone_sync_d = '0') and (phaligndone_sync = '1') then
               phaligndone_cnt    <= phaligndone_cnt( 0) & '1';
            end if;
         else
            phaligndone_cnt       <= "00";
         end if;
      end if;
   end process;

   -----------------------------------------------------------------------------
   -- FSM status
   -----------------------------------------------------------------------------
   RXRESETDONE_FSM                <= '1' when (state = DONE) else '0';
   TXRESETDONE_FSM                <= '1' when (state = DONE) else '0';

end behavioral;

--------------------------------------------------------------------------------
-- GTP PLL
--------------------------------------------------------------------------------
-- Std. library (platform) -----------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- Work library (platform) -----------------------------------------------------
library unisim;
use unisim.vcomponents.all;

entity gtp_pll is
   generic
   (
      --------------------------------------------------------------------------
      -- Simulation
      --------------------------------------------------------------------------
      SIM_RESET_SPEEDUP           : string :=  "TRUE";                        -- Set to "true" to speed up sim reset
      SIM_PLL0REFCLK_SEL          : bit_vector := "001";
      SIM_PLL1REFCLK_SEL          : bit_vector := "001";
      --------------------------------------------------------------------------
      -- PLL0
      --------------------------------------------------------------------------
      PLL0_FBDIV                  : integer := 4;
      PLL0_FBDIV_45               : integer := 5;
      PLL0_REFCLK_DIV             : integer := 1;
      --------------------------------------------------------------------------
      -- PLL1
      --------------------------------------------------------------------------
      PLL1_FBDIV                  : integer := 1;
      PLL1_FBDIV_45               : integer := 4;
      PLL1_REFCLK_DIV             : integer := 1
   );
   port
   (
      --------------------------------------------------------------------------
      -- CPU clock side
      --------------------------------------------------------------------------
      -- System
      CPU_CLK                     : in    std_logic;                          -- Stable Clock
      CPU_RST                     : in    std_logic;                          -- User Reset
      --------------------------------------------------------------------------
      -- Reference clock input
      --------------------------------------------------------------------------
      GTREFCLK0                   : in std_logic;
      GTREFCLK1                   : in std_logic;
      --------------------------------------------------------------------------
      -- PLL0
      --------------------------------------------------------------------------
      -- Reset
      PLL0RESET                   : in std_logic;
      -- Ref clock
      PLL0REFCLKSEL               : in std_logic_vector(2 downto 0);
      PLL0REFCLKLOST              : out std_logic;
      -- Output clock
      PLL0LOCK                    : out std_logic;
      PLL0OUTCLK                  : out std_logic;
      PLL0OUTREFCLK               : out std_logic;
      --------------------------------------------------------------------------
      -- PLL1
      --------------------------------------------------------------------------
      -- Reset
      PLL1RESET                   : in std_logic;
      -- Ref clock
      PLL1REFCLKSEL               : in std_logic_vector(2 downto 0);
      PLL1REFCLKLOST              : out std_logic;
      -- Output clock
      PLL1LOCK                    : out std_logic;
      PLL1OUTCLK                  : out std_logic;
      PLL1OUTREFCLK               : out std_logic
   );
end gtp_pll;

architecture structural of gtp_pll is

   -- CPLL signals
   signal   pllpd_wait            : std_logic_vector( 95 downto  0) := X"FFFFFFFFFFFFFFFFFFFFFFFF";
   signal   pllpd                 : std_logic ;

   attribute equivalent_register_removal: string;
   attribute equivalent_register_removal of pllpd_wait: signal is "no";

begin

   -----------------------------------------------------------------------------
   -- PLL power down
   -----------------------------------------------------------------------------
   process(CPU_CLK)
   begin
      if rising_edge(CPU_CLK) then
         if (CPU_RST = '0') then
            pllpd_wait            <= pllpd_wait(94 downto 0) & '0';
         else
            pllpd_wait            <= X"FFFFFFFFFFFFFFFFFFFFFFFF";
         end if;
      end if;
   end process;

   pllpd                          <= pllpd_wait(95);

   -----------------------------------------------------------------------------
   -- PLL
   -----------------------------------------------------------------------------
   gtpe2_common_inst: GTPE2_COMMON
   generic map
   (
      --------------------------------------------------------------------------
      -- Simulation
      --------------------------------------------------------------------------
      SIM_RESET_SPEEDUP           => SIM_RESET_SPEEDUP,
      SIM_PLL0REFCLK_SEL          => SIM_PLL0REFCLK_SEL,
      SIM_PLL1REFCLK_SEL          => SIM_PLL1REFCLK_SEL,
      SIM_VERSION                 => "2.0",
      --------------------------------------------------------------------------
      -- PLL0
      --------------------------------------------------------------------------
      PLL0_CFG                    => X"01F03DC",
      PLL0_FBDIV                  => PLL0_FBDIV,
      PLL0_FBDIV_45               => PLL0_FBDIV_45,
      PLL0_REFCLK_DIV             => PLL0_REFCLK_DIV,
      PLL0_DMON_CFG               => '0',
      PLL0_INIT_CFG               => X"00001E",
      PLL0_LOCK_CFG               => X"1E8",
      --------------------------------------------------------------------------
      -- PLL1
      --------------------------------------------------------------------------
      PLL1_CFG                    => X"01F03DC",
      PLL1_FBDIV                  => PLL1_FBDIV,
      PLL1_FBDIV_45               => PLL1_FBDIV_45,
      PLL1_REFCLK_DIV             => PLL1_REFCLK_DIV,
      PLL1_DMON_CFG               => '0',
      PLL1_INIT_CFG               => X"00001E",
      PLL1_LOCK_CFG               => X"1E8",
      --------------------------------------------------------------------------
      -- Undocumented
      --------------------------------------------------------------------------
      PLL_CLKOUT_CFG              => X"00",
      COMMON_CFG                  => X"00000000",
      BIAS_CFG                    => X"0000000000050001",
      RSVD_ATTR0                  => X"0000",
      RSVD_ATTR1                  => X"0000"
   )
   port map
   (
      DMONITOROUT             => open,	
      --------------------------------------------------------------------------
      -- DRP
      --------------------------------------------------------------------------
      DRPADDR                     => X"00",
      DRPCLK                      => '0',
      DRPDI                       => X"0000",
      DRPDO                       => open,
      DRPEN                       => '0',
      DRPRDY                      => open,
      DRPWE                       => '0',
      --------------------------------------------------------------------------
      -- Clock input
      --------------------------------------------------------------------------
      GTREFCLK0                   => GTREFCLK0,
      GTREFCLK1                   => GTREFCLK1,

      GTGREFCLK0                  => '0',
      GTGREFCLK1                  => '0',

      GTEASTREFCLK0               => '0',
      GTEASTREFCLK1               => '0',
      GTWESTREFCLK0               => '0',
      GTWESTREFCLK1               => '0',
      --------------------------------------------------------------------------
      -- PLL0
      --------------------------------------------------------------------------
      -- Power down
      PLL0PD                      => pllpd,
      -- Reset
      PLL0RESET                   => PLL0RESET,
      -- Ref clock
      PLL0REFCLKSEL               => PLL0REFCLKSEL,
      PLL0REFCLKLOST              => PLL0REFCLKLOST,
      -- Feedback clock
      PLL0FBCLKLOST               => open,
      -- Output clock
      PLL0LOCKEN                  => '1',
      PLL0LOCKDETCLK              => CPU_CLK,
      PLL0LOCK                    => PLL0LOCK,
      PLL0OUTCLK                  => PLL0OUTCLK,
      PLL0OUTREFCLK               => PLL0OUTREFCLK,
      --------------------------------------------------------------------------
      -- PLL1
      --------------------------------------------------------------------------
      -- Power down
      PLL1PD                      => pllpd,
      -- Reset
      PLL1RESET                   => PLL1RESET,
      -- Ref clock
      PLL1REFCLKSEL               => PLL1REFCLKSEL,
      PLL1REFCLKLOST              => PLL1REFCLKLOST,
      -- Feedback clock
      PLL1FBCLKLOST               => open,
      -- Output clock
      PLL1LOCKEN                  => '1',
      PLL1LOCKDETCLK              => CPU_CLK,
      PLL1LOCK                    => PLL1LOCK,
      PLL1OUTCLK                  => PLL1OUTCLK,
      PLL1OUTREFCLK               => PLL1OUTREFCLK,
      --------------------------------------------------------------------------
      -- Undocumented
      --------------------------------------------------------------------------
      BGRCALOVRDENB               => '1',
      PLLRSVD1                    => "0000000000000000",
      PLLRSVD2                    => "00000",
      REFCLKOUTMONITOR0           => open,
      REFCLKOUTMONITOR1           => open,
      BGBYPASSB                   => '1',
      BGMONITORENB                => '1',
      BGPDB                       => '1',
      BGRCALOVRD                  => "11111",
      PMARSVD                     => "00000000",
      PMARSVDOUT                  => open,
      RCALENB                     => '1'
   );

end structural;

--------------------------------------------------------------------------------
-- GTPE2 wrapper.
--------------------------------------------------------------------------------
-- Std. library (platform) -----------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- Work library (platform) -----------------------------------------------------
library unisim;
use unisim.vcomponents.all;

-- Work library (application) --------------------------------------------------
use work.gtp_link_package.all;

entity gtp_link is
   generic
   (
      --------------------------------------------------------------------------
      -- Simulation attributes
      --------------------------------------------------------------------------
      SIM_RESET_SPEEDUP           : boolean:= FALSE;                          -- Set to "TRUE" to speed up sim reset
      --------------------------------------------------------------------------
      -- Startup timing
      --------------------------------------------------------------------------
      CPU_CLK_MHz                 : real := 125.0;                            -- Frequency of the stable clock in [MHz]
      REF_CLK_MHz                 : real := 125.0;                            -- Frequency of the reference clock in [MHz]
      BAUD_RATE_Gbps              : real := 5.0                               -- Baudrate in GBit/s
   );
   port
   (
      debug_clk                   : out   std_logic;
      debug                       : out   std_logic_vector(127 downto  0);

      i_gtp_ctrl                  : in    gtp_ctrl_in_type;
      i_gtp                       : in    gtp_in_type;
      o_gtp                       : out   gtp_out_type
   );
end gtp_link;

architecture structural of gtp_link is

   -- DRP signals
   signal   drpclk                : std_ulogic;
   signal   drpen                 : std_ulogic;
   signal   drpwe                 : std_ulogic;
   signal   drpaddr               : std_logic_vector( 8 downto  0);
   signal   drpdi                 : std_logic_vector(15 downto  0);
   signal   drpdo                 : std_logic_vector(15 downto  0);
   signal   drprdy                : std_ulogic;
   -- Receiver signals
   signal   gtrxreset             : std_logic;
   signal   rxpmaresetdone        : std_logic;
   signal   rxresetdone           : std_logic;
   signal   rxcdrlock             : std_logic;
   signal   rxoutclk              : std_logic;
   signal   rxusrclk              : std_logic;
   signal   rxdlysreset           : std_logic;
   signal   rxdlysresetdone       : std_logic;
   signal   rxphaligndone         : std_logic;
   signal   rxuserrdy             : std_logic;
   signal   rxdata                : std_logic_vector(31 downto  0);
   signal   rxcharisk             : std_logic_vector( 3 downto  0);
   signal   rxdisperr             : std_logic_vector( 3 downto  0);
   signal   rxnotintable          : std_logic_vector( 3 downto  0);
   -- Transmitter signals
   signal   gttxreset             : std_logic;
   signal   txresetdone           : std_logic;
   signal   txoutclk              : std_logic;
   signal   txusrclk              : std_logic;
   signal   txuserrdy             : std_logic;
   signal   txdata                : std_logic_vector(31 downto  0);
   signal   txcharisk             : std_logic_vector( 3 downto  0);

   constant RXOUT_DIV             : integer := PARAM_RXTXOUT_DIV(REF_CLK_MHz, BAUD_RATE_Gbps);
   constant TXOUT_DIV             : integer := PARAM_RXTXOUT_DIV(REF_CLK_MHz, BAUD_RATE_Gbps);
   constant USR_CLK_MHz           : real := PARAM_USRCLK(BAUD_RATE_Gbps);

   --ML84
   signal rxbyteisaligned, rxbyterealign, rxcommadet : std_logic;

begin

   -----------------------------------------------------------------------------
   -- Debug
   -----------------------------------------------------------------------------
   debug_clk                      <= rxusrclk;
   debug(  0)                     <= rxcdrlock;
   debug(  2 downto 1)            <= rxdisperr( 1 downto  0);
   debug(  4 downto 3)            <= rxnotintable( 1 downto  0);
   debug(  5)                     <= rxuserrdy;
   debug(  6)                     <= rxphaligndone;
   debug(  7)                     <= rxdlysreset;
   debug(  8)                     <= rxdlysresetdone;
   debug( 15 downto   9)          <= "0000000";
   debug( 31 downto  16)          <= rxdata(15 downto  0);
   debug( 33 downto  32)          <= rxcharisk( 1 downto  0);
   debug( 39 downto  34)          <= "000000";
   debug(127 downto  40)          <= X"0000000000000000000000";

   -----------------------------------------------------------------------------
   -- User clocks
   -----------------------------------------------------------------------------
   rxoutclk_bufg1_i : BUFG
   port map
   (
      I                           => rxoutclk,
      O                           => rxusrclk
   );

   o_gtp.rx.rxusrclk              <= rxusrclk;

   txusrclk_bufg_inst: BUFG
   port map
   (
      I                           => txoutclk,
      O                           => txusrclk
   );

   o_gtp.tx.txusrclk              <= txusrclk;

   -----------------------------------------------------------------------------
   -- Start-up FSM
   -----------------------------------------------------------------------------
   gtp_startup_inst: gtp_startup
   generic map
   (
      SIM_RESET_SPEEDUP           => SIM_RESET_SPEEDUP,
      RX_PLL0_USED                => TRUE,
      TX_PLL0_USED                => TRUE,
      RX_PLL1_USED                => FALSE,
      TX_PLL1_USED                => FALSE,
      CPU_CLK_MHz                 => CPU_CLK_MHz,
      USR_CLK_MHz                 => USR_CLK_MHz,
      BAUD_RATE_Gbps              => BAUD_RATE_Gbps
   )
   port map
   (
      --------------------------------------------------------------------------
      -- CPU clock side
      --------------------------------------------------------------------------
      -- System
      CPU_CLK                     => i_gtp_ctrl.clk,
      CPU_RST                     => i_gtp_ctrl.rst,
      -- PLL
      PLL0RESET                   => o_gtp.pll.pll0reset,
      PLL0LOCK                    => i_gtp.pll.pll0lock,
      PLL1RESET                   => o_gtp.pll.pll1reset,
      PLL1LOCK                    => i_gtp.pll.pll1lock,
      -- Resets
      GTRXRESET                   => gtrxreset,
      GTTXRESET                   => gttxreset,
      -- Status
      RXUSERRDY                   => rxuserrdy,
      TXUSERRDY                   => txuserrdy,
      RXRESETDONE_FSM             => o_gtp.rx.rxresetdone,
      TXRESETDONE_FSM             => o_gtp.tx.txresetdone,
      --------------------------------------------------------------------------
      -- DRP clock side
      --------------------------------------------------------------------------
      DRPCLK                      => drpclk,
      DRPEN                       => drpen,
      DRPRDY                      => drprdy,
      DRPWE                       => drpwe,
      DRPADDR                     => drpaddr,
      DRPDI                       => drpdi,
      DRPDO                       => drpdo,
      --------------------------------------------------------------------------
      -- MGT clock side
      --------------------------------------------------------------------------
      -- GTP reset
      RXPMARESETDONE              => rxpmaresetdone,   
      RXRESETDONE                 => rxresetdone,
      TXRESETDONE                 => txresetdone,
      -- RX delay alignment reset
      RXCDRLOCK                   => rxcdrlock,
      RXDLYSRESET                 => rxdlysreset,
      RXDLYSRESETDONE             => rxdlysresetdone,
      PHALIGNDONE                 => rxphaligndone
   );

   -----------------------------------------------------------------------------
   -- Receiver mapping
   -----------------------------------------------------------------------------
   o_gtp.rx.rxcharisk             <= rxcharisk(1 downto 0);
   o_gtp.rx.rxdata                <= rxdata(15 downto 0);
   o_gtp.rx.rxdisperr             <= rxdisperr(1 downto 0);
   o_gtp.rx.rxnotintable          <= rxnotintable(1 downto 0);
   o_gtp.rx.rxbyteisaligned       <= rxbyteisaligned;
   o_gtp.rx.rxbyterealign         <= rxbyterealign;  
   o_gtp.rx.rxcommadet            <= rxcommadet;     

   -----------------------------------------------------------------------------
   -- Transmitter mapping
   -----------------------------------------------------------------------------
   txdata                         <= X"0000" & i_gtp.tx.txdata;
   txcharisk                      <= "00" & i_gtp.tx.txcharisk;

   -----------------------------------------------------------------------------
   -- GTPE2 instance
   -----------------------------------------------------------------------------
   gtpe2_channel_inst : GTPE2_CHANNEL
   generic map
   (
      --------------------------------------------------------------------------
      -- Simulation
      --------------------------------------------------------------------------
      SIM_VERSION                 => "2.0",
      --------------------------------------------------------------------------
      -- JTAG
      --------------------------------------------------------------------------
      ACJTAG_DEBUG_MODE           => '0',
      ACJTAG_MODE                 => '0',
      ACJTAG_RESET                => '0',
      --------------------------------------------------------------------------
      -- Reset
      --------------------------------------------------------------------------
      -- Simulation
      SIM_RESET_SPEEDUP           => "TRUE",
      -- Reserved
      RXOSCALRESET_TIME           => "00011",
      RXOSCALRESET_TIMEOUT        => "00000",
      --------------------------------------------------------------------------
      -- Power down (PCIexpress cases)
      --------------------------------------------------------------------------
      PD_TRANS_TIME_FROM_P2       => X"03c",
      PD_TRANS_TIME_NONE_P2       => X"3c",
      PD_TRANS_TIME_TO_P2         => X"64",
      --------------------------------------------------------------------------
      -- Clock
      --------------------------------------------------------------------------
      OUTREFCLK_SEL_INV           => "11",
      CLK_COMMON_SWING            => '0',
      --------------------------------------------------------------------------
      -- Rate select
      --------------------------------------------------------------------------
      TRANS_TIME_RATE             => X"0E",
      --------------------------------------------------------------------------
      -- Loopback
      --------------------------------------------------------------------------
      LOOPBACK_CFG                => '0',
      PMA_LOOPBACK_CFG            => '0',
      --------------------------------------------------------------------------
      -- Digital monitor
      --------------------------------------------------------------------------
      RX_DEBUG_CFG                => "00000000000000",
      DMONITOR_CFG                => X"000A00",
      --------------------------------------------------------------------------
      -- Gearbox
      --------------------------------------------------------------------------
      GEARBOX_MODE                => "000",
      --------------------------------------------------------------------------
      -- Special OOB Signalling
      --------------------------------------------------------------------------
      SATA_PLL_CFG                => "VCO_3000MHZ",
      SATA_BURST_SEQ_LEN          => "0101",
      SATA_BURST_VAL              => "100",
      SATA_EIDLE_VAL              => "100",
      SATA_MIN_BURST              => 4,
      SATA_MIN_INIT               => 12,
      SATA_MIN_WAKE               => 4,
      SATA_MAX_BURST              => 8,
      SATA_MAX_INIT               => 21,
      SATA_MAX_WAKE               => 7,
      SAS_MIN_COM                 => 36,
      SAS_MAX_COM                 => 64,
      PCS_RSVD_ATTR               => X"000000000000",
      --------------------------------------------------------------------------
      -- Special PCIexpress settings
      --------------------------------------------------------------------------
      TX_RXDETECT_CFG             => X"1832",
      TX_RXDETECT_REF             => "100",
      --------------------------------------------------------------------------
      -- Special undocumented settings
      --------------------------------------------------------------------------
      PMA_RSV                     => X"00000333",
      PMA_RSV2                    => X"00002040",
      PMA_RSV3                    => "00",
      PMA_RSV4                    => "0000",
      PMA_RSV5                    => '0',
      PMA_RSV6                    => '0',
      PMA_RSV7                    => '0',
      RX_BIAS_CFG                 => "0000111100110011",
      RX_CLK25_DIV                => 5,
      TX_CLK25_DIV                => 5,
      RX_CLKMUX_EN                => '1',
      TX_CLKMUX_EN                => '1',
      --------------------------------------------------------------------------
      -- Receiver
      --------------------------------------------------------------------------
      -- Simulation
      SIM_RECEIVER_DETECT_PASS    => "TRUE",
      -- Reset
      RXPMARESET_TIME             => "00011",
      RXCDRPHRESET_TIME           => "00001",
      RXCDRFREQRESET_TIME         => "00001",
      RXISCANRESET_TIME           => "00001",
      RXPCSRESET_TIME             => "00001",
      -- AFE (Analog Front End)
      RX_CM_SEL                   => "11",
      RX_CM_TRIM                  => "1010",
      TERM_RCAL_CFG               => "100001000010000",
      TERM_RCAL_OVRD              => "000",
      RXLPM_INCM_CFG              => '1',
      RXLPM_IPCM_CFG              => '0',
      -- OOB (Out Of Band signalling)
      RXOOB_CFG                   => "0000110",
      RXOOB_CLK_CFG               => "PMA",
      -- LPM (Low Power Mode equalizer)
      ADAPT_CFG0                  => X"00000",
      RXLPM_HOLD_DURING_EIDLE     => '0',
      RXLPMRESET_TIME             => "0001111",
      RXLPM_CM_CFG                => '0',
      RXLPM_CFG                   => "0110",
      RXLPM_CFG1                  => '0',
      RXLPM_LF_CFG                => "000000001111110000",
      RXLPM_LF_CFG2               => "01010",
      RXLPM_HF_CFG                => "00001111110000",
      RXLPM_HF_CFG2               => "01010",
      RXLPM_HF_CFG3               => "0000",
      RXLPM_GC_CFG                => "111100010",
      RXLPM_GC_CFG2               => "001",
      RXLPM_OSINT_CFG             => "100",
      RXLPM_BIAS_STARTUP_DISABLE  => '0',
      -- CDR (Clock Data Recovery)
      CFOK_CFG                    => X"49000040E80",
      CFOK_CFG2                   => "0100000",
      CFOK_CFG3                   => "0100000",
      CFOK_CFG4                   => '0',
      CFOK_CFG5                   => X"0",
      CFOK_CFG6                   => "0000",

      RXCDR_CFG                   => X"0001107FE206021041010",
      RXCDR_LOCK_CFG              => "001001",
      RXCDR_HOLD_DURING_EIDLE     => '0',
      RXCDR_FR_RESET_ON_EIDLE     => '0',
      RXCDR_PH_RESET_ON_EIDLE     => '0',

      RX_OS_CFG                   => "0000010000000",
      -- Fabric Clock Output Control
      RXOUT_DIV                   => RXOUT_DIV,
      -- Margin Analysis
      ES_VERT_OFFSET              => "000000000",
      ES_HORZ_OFFSET              => X"000",
      ES_PRESCALE                 => "00000",
      ES_SDATA_MASK               => X"00000000000000000000",
      ES_QUALIFIER                => X"00000000000000000000",
      ES_QUAL_MASK                => X"00000000000000000000",
      ES_EYE_SCAN_EN              => "TRUE",
      ES_ERRDET_EN                => "FALSE",
      ES_PMA_CFG                  => "0000000000",
      ES_CONTROL                  => "000000",
      USE_PCS_CLK_PHASE_SEL       => '0',
      ES_CLK_PHASE_SEL            => '0',
      -- Pattern Checker
      RXPRBS_ERR_LOOPBACK         => '0',
      -- Byte and Word Alignment
      ALIGN_COMMA_WORD            => 2,
      ALIGN_COMMA_ENABLE          => "0011111111",
      ALIGN_COMMA_DOUBLE          => "FALSE",
      ALIGN_MCOMMA_VALUE          => "1010000011",
      ALIGN_MCOMMA_DET            => "TRUE",
      ALIGN_PCOMMA_VALUE          => "0101111100",
      ALIGN_PCOMMA_DET            => "TRUE",
      SHOW_REALIGN_COMMA          => "TRUE",
      RXSLIDE_MODE                => "PCS",
      RXSLIDE_AUTO_WAIT           => 7,
      RX_SIG_VALID_DLY            => 10,
      -- 8B/10B Decoder
      RX_DISPERR_SEQ_MATCH        => "TRUE",
      DEC_PCOMMA_DETECT           => "TRUE",
      DEC_MCOMMA_DETECT           => "TRUE",
      DEC_VALID_COMMA_ONLY        => "FALSE",
      UCODEER_CLR                 => '0',
      -- Buffer Bypass
      RXPH_CFG                    => X"C00002",
      RXPH_MONITOR_SEL            => "00000",
      RXPHDLY_CFG                 => X"084020",

      RXDLY_CFG                   => X"001F",
      RXDLY_LCFG                  => X"030",
      RXDLY_TAP_CFG               => X"0000",

      RX_DDI_SEL                  => "000000",

      RXSYNC_MULTILANE            => '0',
      RXSYNC_SKIP_DA              => '0',
      RXSYNC_OVRD                 => '0',

      TST_RSV                     => X"00000000",
      -- Elastic Buffer
      RXBUF_RESET_ON_RATE_CHANGE  => "TRUE",
      RXBUF_EN                    => "TRUE",
      RX_XCLK_SEL                 => "RXUSR",
      RX_BUFFER_CFG               => "000000",
      RX_DEFER_RESET_BUF_EN       => "TRUE",
      RXBUF_ADDR_MODE             => "FAST",
      RXBUF_EIDLE_HI_CNT          => "1000",
      RXBUF_EIDLE_LO_CNT          => "0000",
      RXBUF_RESET_ON_CB_CHANGE    => "TRUE",
      RXBUF_RESET_ON_COMMAALIGN   => "FALSE",
      RXBUF_RESET_ON_EIDLE        => "FALSE",
      RXBUF_THRESH_OVRD           => "FALSE",
      RXBUF_THRESH_OVFLW          => 61,
      RXBUF_THRESH_UNDFLW         => 4,
      RXBUFRESET_TIME             => "00001",
      -- Clock Correction
      CBCC_DATA_SOURCE_SEL        => "DECODED",
      CLK_CORRECT_USE             => "FALSE",
      CLK_COR_KEEP_IDLE           => "FALSE",
      CLK_COR_MAX_LAT             => 10,
      CLK_COR_MIN_LAT             => 8,
      CLK_COR_PRECEDENCE          => "TRUE",
      CLK_COR_REPEAT_WAIT         => 0,
      CLK_COR_SEQ_LEN             => 1,
      CLK_COR_SEQ_1_ENABLE        => "1111",
      CLK_COR_SEQ_1_1             => "0100000000",
      CLK_COR_SEQ_1_2             => "0000000000",
      CLK_COR_SEQ_1_3             => "0000000000",
      CLK_COR_SEQ_1_4             => "0000000000",
      CLK_COR_SEQ_2_USE           => "FALSE",
      CLK_COR_SEQ_2_ENABLE        => "1111",
      CLK_COR_SEQ_2_1             => "0100000000",
      CLK_COR_SEQ_2_2             => "0000000000",
      CLK_COR_SEQ_2_3             => "0000000000",
      CLK_COR_SEQ_2_4             => "0000000000",
      -- Channel Bonding
      CHAN_BOND_MAX_SKEW          => 1,
      CHAN_BOND_KEEP_ALIGN        => "FALSE",
      CHAN_BOND_SEQ_1_1           => "0000000000",
      CHAN_BOND_SEQ_1_2           => "0000000000",
      CHAN_BOND_SEQ_1_3           => "0000000000",
      CHAN_BOND_SEQ_1_4           => "0000000000",
      CHAN_BOND_SEQ_1_ENABLE      => "1111",
      CHAN_BOND_SEQ_2_1           => "0000000000",
      CHAN_BOND_SEQ_2_2           => "0000000000",
      CHAN_BOND_SEQ_2_3           => "0000000000",
      CHAN_BOND_SEQ_2_4           => "0000000000",
      CHAN_BOND_SEQ_2_ENABLE      => "1111",
      CHAN_BOND_SEQ_2_USE         => "FALSE",
      CHAN_BOND_SEQ_LEN           => 1,

      FTS_DESKEW_SEQ_ENABLE       => "1111",
      FTS_LANE_DESKEW_CFG         => "1111",
      FTS_LANE_DESKEW_EN          => "FALSE",
      -- Phase Interpolator PPM Controller
      RXPI_CFG0                   => "000",
      RXPI_CFG1                   => '1',
      RXPI_CFG2                   => '1',
      -- PCIexpress
      PCS_PCIE_EN                 => "FALSE",
      -- Gearbox
      RXGEARBOX_EN                => "FALSE",
      -- FPGA Interface
      RX_DATA_WIDTH               => 20,
      --------------------------------------------------------------------------
      -- Transmitter
      --------------------------------------------------------------------------
      -- Simulation
      SIM_TX_EIDLE_DRIVE_LEVEL    => "X",
      -- Reset
      TXPMARESET_TIME             => "00001",
      TXPCSRESET_TIME             => "00001",
      -- FPGA Interface
      TX_DATA_WIDTH               => 20,
      -- Gearbox
      TXGEARBOX_EN                => "FALSE",
      -- Buffer
      TXBUF_EN                    => "TRUE",
      TXBUF_RESET_ON_RATE_CHANGE  => "TRUE",
      TX_XCLK_SEL                 => "TXOUT",
      -- Buffer bypass
      TXPH_CFG                    => X"0780",
      TXPH_MONITOR_SEL            => "00000",
      TXPHDLY_CFG                 => X"084020",
      TXDLY_CFG                   => X"001F",
      TXDLY_LCFG                  => X"030",
      TXDLY_TAP_CFG               => X"0000",
      TXSYNC_MULTILANE            => '0',
      TXSYNC_SKIP_DA              => '0',
      TXSYNC_OVRD                 => '1',
      -- Fabric Clock Output Control
      TXOUT_DIV                   => TXOUT_DIV,
      -- Phase Interpolator PPM Controller
      TXPI_SYNFREQ_PPM            => "001",
      TXPI_PPM_CFG                => X"00",
      TXPI_INVSTROBE_SEL          => '0',
      TXPI_GREY_SEL               => '0',
      TXPI_PPMCLK_SEL             => "TXUSRCLK2",
      TXPI_CFG0                   => "00",
      TXPI_CFG1                   => "00",
      TXPI_CFG2                   => "00",
      TXPI_CFG3                   => '0',
      TXPI_CFG4                   => '0',
      TXPI_CFG5                   => "000",
      -- Driver
      TX_DEEMPH0                  => "000000",
      TX_DEEMPH1                  => "000000",
      TX_DRIVE_MODE               => "DIRECT",
      TX_MAINCURSOR_SEL           => '0',
      TX_MARGIN_FULL_0            => "1001110",
      TX_MARGIN_FULL_1            => "1001001",
      TX_MARGIN_FULL_2            => "1000101",
      TX_MARGIN_FULL_3            => "1000010",
      TX_MARGIN_FULL_4            => "1000000",
      TX_MARGIN_LOW_0             => "1000110",
      TX_MARGIN_LOW_1             => "1000100",
      TX_MARGIN_LOW_2             => "1000010",
      TX_MARGIN_LOW_3             => "1000000",
      TX_MARGIN_LOW_4             => "1000000",
      TX_PREDRIVER_MODE           => '0',
      TX_EIDLE_ASSERT_DELAY       => "110",
      TX_EIDLE_DEASSERT_DELAY     => "100",
      TX_LOOPBACK_DRIVE_HIZ       => "FALSE",
      -- OOB (Out Of Band signalling)
      TXOOB_CFG                   => '0'
   )
   port map
   (
      --------------------------------------------------------------------------
      -- Reset
      --------------------------------------------------------------------------
      GTRESETSEL                  => '0',
      RESETOVRD                   => '0',
      --------------------------------------------------------------------------
      -- Clock
      --------------------------------------------------------------------------
      -- Sources
      PLL0CLK                     => i_gtp.pll.pll0clk,
      PLL0REFCLK                  => i_gtp.pll.pll0refclk,
      PLL1CLK                     => i_gtp.pll.pll1clk,
      PLL1REFCLK                  => i_gtp.pll.pll1refclk,
      SIGVALIDCLK                 => '0',
      --------------------------------------------------------------------------
      -- CPLL
      --------------------------------------------------------------------------
      -- Reserved
      GTRSVD                      => "0000000000000000",
      PCSRSVDIN                   => "0000000000000000",
      TSTIN                       => "11111111111111111111",
     --------------------------------------------------------------------------
      -- Loopback
      --------------------------------------------------------------------------
      LOOPBACK                    => "000",
      --------------------------------------------------------------------------
      -- DRP
      --------------------------------------------------------------------------
      DRPCLK                      => drpclk,
      DRPEN                       => drpen,
      DRPADDR                     => drpaddr,
      DRPWE                       => drpwe,
      DRPDI                       => drpdi,
      DRPDO                       => drpdo,
      DRPRDY                      => drprdy,
      --------------------------------------------------------------------------
      -- Digital monitor
      --------------------------------------------------------------------------
      DMONITORCLK                 => '0',
      DMONFIFORESET               => '0',
      DMONITOROUT                 => open,
      --------------------------------------------------------------------------
      -- Receiver
      --------------------------------------------------------------------------
      -- Power down
      RXPD                        => i_gtp.rx.rxpd,
      -- Clock
      RXSYSCLKSEL                 => "00",
      -- Reset
      GTRXRESET                   => gtrxreset,
      RXPMARESET                  => '0',
      RXPMARESETDONE              => rxpmaresetdone,
      RXPCSRESET                  => '0',
      RXRESETDONE                 => rxresetdone,

      RXUSERRDY                   => rxuserrdy,
      RXUSRCLK                    => rxusrclk,
      RXUSRCLK2                   => rxusrclk,
      -- Serial
      GTPRXP                      => i_gtp.rx.gtprxp,
      GTPRXN                      => i_gtp.rx.gtprxn,
      -- AFE (Analog Front End)
      PMARSVDIN2                  => '0',
      PMARSVDOUT0                 => open,
      PMARSVDOUT1                 => open,
      -- OOB (Out Of Band signalling)
      RXOOBRESET                  => '0',
      RXCOMINITDET                => open,
      RXCOMSASDET                 => open,
      RXCOMWAKEDET                => open,
      RXELECIDLE                  => open,
      RXELECIDLEMODE              => "11",
      -- LPM (Low Power Mode equalizer)
      RXLPMRESET                  => '0',
      RXLPMHFHOLD                 => '0',
      RXLPMHFOVRDEN               => '0',
      RXLPMLFHOLD                 => '0',
      RXLPMLFOVRDEN               => '0',
      RXLPMOSINTNTRLEN            => '0',
      -- DFE (Decision Feedback Equalization)
      RXADAPTSELTEST              => "00000000000000",
      RXDFEXYDEN                  => '0',
      RXOSINTEN                   => '1',
      RXOSINTID0                  => "0000",
      RXOSINTNTRLEN               => '0',
      RXOSINTSTROBEDONE           => open,
      -- CDR (Clock Data Recovery)
      RXCDRRESET                  => '0',
      RXCDRRESETRSV               => '0',
      RXCDRFREQRESET              => '0',
      RXCDRHOLD                   => '0',
      RXCDROVRDEN                 => '0',
      RXCDRLOCK                   => rxcdrlock,

      RXOSHOLD                    => '0',
      RXOSOVRDEN                  => '0',
      RXOSCALRESET                => '0',

      RXOSINTPD                   => '0',
      RXOSINTCFG                  => "0010",
      RXOSINTOVRDEN               => '0',
      RXOSINTSTROBE               => '0',
      RXOSINTHOLD                 => '0',
      RXOSINTTESTOVRDEN           => '0',
      RXOSINTSTARTED              => open,
      RXOSINTSTROBESTARTED        => open,
      RXOSINTDONE                 => open,
      -- Fabric Clock Output Control
      RXRATEMODE                  => '0',
      RXRATE                      => "000",
      RXRATEDONE                  => open,

      RXOUTCLKPCS                 => open,
      RXOUTCLKFABRIC              => open,
      RXOUTCLKSEL                 => "010",
      RXOUTCLK                    => rxoutclk,
      -- Margin Analysis
      EYESCANRESET                => '0',
      EYESCANMODE                 => '0',
      EYESCANTRIGGER              => '0',
      EYESCANDATAERROR            => open,
      -- Polarity Control
      RXPOLARITY                  => '0',
      -- Pattern Checker
      RXPRBSCNTRESET              => '0',
      RXPRBSSEL                   => "000",
      RXPRBSERR                   => open,
      -- Byte and Word Alignment
      RXBYTEISALIGNED             => rxbyteisaligned,
      RXBYTEREALIGN               => rxbyterealign,
      RXCOMMADET                  => rxcommadet,
      RXCOMMADETEN                => '1',
      RXPCOMMAALIGNEN             => '1',
      RXMCOMMAALIGNEN             => '1',

      RXSLIDE                     => '0',
      -- 8B/10B Decoder
      RX8B10BEN                   => '1',
      RXCHARISCOMMA               => open,
      RXCHARISK                   => rxcharisk,
      RXDISPERR                   => rxdisperr,
      RXNOTINTABLE                => rxnotintable,
      SETERRSTATUS                => '0',
      -- Buffer Bypass
      RXPHDLYPD                   => '0',
      RXPHDLYRESET                => '0',
      RXPHOVRDEN                  => '0',

      RXPHALIGN                   => '0',
      RXPHALIGNEN                 => '0',
      RXPHALIGNDONE               => rxphaligndone,

      RXDLYSRESET                 => rxdlysreset,
      RXDLYSRESETDONE             => rxdlysresetdone,
      RXDLYBYPASS                 => '0',
      RXDLYEN                     => '0',
      RXDLYOVRDEN                 => '0',
      RXDDIEN                     => '1',

      RXSYNCMODE                  => '1',
      RXSYNCALLIN                 => rxphaligndone,
      RXSYNCIN                    => '0',
      RXSYNCOUT                   => open,
      RXSYNCDONE                  => open,

      RXPHMONITOR                 => open,
      RXPHSLIPMONITOR             => open,
      -- Elastic Buffer
      RXBUFRESET                  => '0',
      RXBUFSTATUS                 => open,
      -- Clock Correction
      RXCLKCORCNT                 => open,
      -- Channel Bonding
      RXCHANISALIGNED             => open,
      RXCHANREALIGN               => open,

      RXCHANBONDSEQ               => open,
      RXCHBONDI                   => "0000",
      RXCHBONDO                   => open,
      RXCHBONDLEVEL               => "000",
      RXCHBONDMASTER              => '0',
      RXCHBONDSLAVE               => '0',
      RXCHBONDEN                  => '0',
      -- Gearbox
      RXDATAVALID                 => open,
      RXGEARBOXSLIP               => '0',
      RXHEADER                    => open,
      RXHEADERVALID               => open,
      RXSTARTOFSEQ                => open,
      -- PCIexpress
      RXVALID                     => open,
      PHYSTATUS                   => open,
      RXSTATUS                    => open,
      -- FPGA Interface
      RXDATA                      => rxdata,
      --------------------------------------------------------------------------
      -- Transmitter
      --------------------------------------------------------------------------
      -- Power down
      TXPD                        => i_gtp.tx.txpd,
      -- Clock
      TXSYSCLKSEL                 => "00",
      -- Reset
      GTTXRESET                   => gttxreset,
      TXPMARESET                  => '0',
      TXPMARESETDONE              => open,
      TXPCSRESET                  => '0',
      TXRESETDONE                 => txresetdone,
      -- Reset reserved
      CFGRESET                    => '0',
      PCSRSVDOUT                  => open,
      -- FPGA Interface
      TXUSERRDY                   => txuserrdy,
      TXUSRCLK                    => txusrclk,
      TXUSRCLK2                   => txusrclk,
      TXDATA                      => txdata,
      -- 8B/10B Encoder
      TX8B10BBYPASS               => "0000",
      TX8B10BEN                   => '1',
      TXCHARDISPMODE              => "0000",
      TXCHARDISPVAL               => "0000",
      TXCHARISK                   => txcharisk,
      -- Gearbox
      TXGEARBOXREADY              => open,
      TXHEADER                    => "000",
      TXSEQUENCE                  => "0000000",
      TXSTARTSEQ                  => '0',
      -- Buffer
      TXBUFSTATUS                 => open,
      -- Buffer bypass
      TXPHDLYPD                   => '0',
      TXPHDLYRESET                => '0',
      TXPHDLYTSTCLK               => '0',

      TXPHALIGN                   => '0',
      TXPHALIGNEN                 => '0',
      TXPHALIGNDONE               => open,

      TXPHINIT                    => '0',
      TXPHINITDONE                => open,
      TXPHOVRDEN                  => '0',

      TXDLYSRESET                 => '0',
      TXDLYSRESETDONE             => open,
      TXDLYBYPASS                 => '1',
      TXDLYEN                     => '0',
      TXDLYOVRDEN                 => '0',
      TXDLYHOLD                   => '0',
      TXDLYUPDOWN                 => '0',

      TXSYNCMODE                  => '0',
      TXSYNCALLIN                 => '0',
      TXSYNCIN                    => '0',
      TXSYNCOUT                   => open,
      TXSYNCDONE                  => open,
      -- Pattern Generator
      TXPRBSSEL                   => "000",
      TXPRBSFORCEERR              => '0',
      -- Polarity Control
      TXPOLARITY                  => '0',
      -- Fabric Clock Output Control
      TXRATEMODE                  => '0',
      TXRATE                      => "000",
      TXRATEDONE                  => open,

      TXOUTCLKPCS                 => open,
      TXOUTCLKFABRIC              => open,
      TXOUTCLKSEL                 => "010",
      TXOUTCLK                    => txoutclk,
      -- Phase Interpolator PPM Controller
      TXPIPPMPD                   => '0',
      TXPIPPMEN                   => '0',
      TXPIPPMOVRDEN               => '0',
      TXPIPPMSEL                  => '1',
      TXPIPPMSTEPSIZE             => "00000",
      -- PCIexpress
      TXDETECTRX                  => '0',
      -- Driver
      TXINHIBIT                   => '0',
      TXELECIDLE                  => '0',
      TXDIFFCTRL                  => "1000",
      TXBUFDIFFCTRL               => "100",
      TXMAINCURSOR                => "0000000",
      TXDEEMPH                    => '0',
      TXMARGIN                    => "000",
      PMARSVDIN0                  => '0',
      PMARSVDIN1                  => '0',

      TXPRECURSOR                 => "00000",
      TXPRECURSORINV              => '0',

      TXPOSTCURSOR                => "00000",
      TXPOSTCURSORINV             => '0',

      TXSWING                     => '0',

      TXDIFFPD                    => '0',
      TXPISOPD                    => '0',
      -- OOB (Out Of Band signalling)
      TXCOMFINISH                 => open,
      TXCOMINIT                   => '0',
      TXCOMSAS                    => '0',
      TXCOMWAKE                   => '0',
      TXPDELECIDLEMODE            => '0',
      -- Serial
      GTPTXN                      => o_gtp.tx.gtptxn,
      GTPTXP                      => o_gtp.tx.gtptxp,
      --------------------------------------------------------------------------
      -- Unknown
      --------------------------------------------------------------------------
      CLKRSVD0                    => '0',
      CLKRSVD1                    => '0',
      PMARSVDIN3                  => '0',
      PMARSVDIN4                  => '0'
   );

 end structural;

--------------------------------------------------------------------------------
-- End of file
--------------------------------------------------------------------------------
