--------------------------------------------------------------------------------
--                       Paul Scherrer Institute (PSI)
--------------------------------------------------------------------------------
-- Unit    : axis_gtp_bridge_v1_0.vhd
-- Author  : Goran Marinkovic, Section Diagnostic
-- Version : $Revision: 1.3 $
--------------------------------------------------------------------------------
-- Copyright© PSI, Section Diagnostic
--------------------------------------------------------------------------------
-- Comment : This is the axis interface for the GTPE2 component.
--------------------------------------------------------------------------------
-- Std. library (platform) -----------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- Work library (application) --------------------------------------------------
use work.gtp_link_package.all;
use work.gtp_ipif_package.all;

entity axis_gtp_bridge_v1_0 is
   generic
   (
      --------------------------------------------------------------------------
      -- Simulation attributes
      --------------------------------------------------------------------------
      SIM_RESET_SPEEDUP           : boolean := FALSE;                         -- Set to "TRUE" to speed up sim reset
      --------------------------------------------------------------------------
      -- GTP timing
      --------------------------------------------------------------------------
      CPU_CLK_Hz                  : integer := 125000000;                      -- Frequency of the stable clock in [Hz]
      REF_CLK_Hz                  : integer := 125000000;                      -- Frequency of the reference clock in [Hz]
      BAUD_RATE_Mbps              : integer := 2500;                           -- Baudrate in MBit/s
      --------------------------------------------------------------------------
      -- AXIS interface
      --------------------------------------------------------------------------
      -- AXIS slave bus interface
      C_S00_AXIS_TDATA_WIDTH      : integer := 32;
      -- AXIS master bus interface
      C_M00_AXIS_TDATA_WIDTH      : integer := 32
   );
   port
   (
      --------------------------------------------------------------------------
      -- Debug
      --------------------------------------------------------------------------
      debug_clk                    : out    std_logic;
      debug                        : out    std_logic_vector(127 downto  0);
      --------------------------------------------------------------------------
      -- System
      --------------------------------------------------------------------------
      aclk                        : in    std_logic;
      aresetn                     : in    std_logic;
      --------------------------------------------------------------------------
      -- GTP interface
      --------------------------------------------------------------------------
      i_clk_gtp                   : in    std_logic_vector( 1 downto  0);

      i_gtp_rx_p                  : in    std_logic;
      i_gtp_rx_n                  : in    std_logic;
      o_gtp_tx_p                  : out   std_logic;
      o_gtp_tx_n                  : out   std_logic;

      o_lock_plb                  : out   std_logic;
      o_lock_axi                  : out   std_logic;
      o_lock_comma                : out   std_logic;
      --------------------------------------------------------------------------
      -- AXIS interface
      --------------------------------------------------------------------------
      -- AXIS slave bus interface
      s00_axis_tready             : out   std_logic;
      s00_axis_tdata              : in    std_logic_vector(C_S00_AXIS_TDATA_WIDTH - 1 downto  0);
      s00_axis_tuser              : in    std_logic_vector((C_S00_AXIS_TDATA_WIDTH / 8) - 1 downto  0);
      s00_axis_tvalid             : in    std_logic;

      -- AXIS master bus interface
      m00_axis_tvalid             : out   std_logic;
      m00_axis_tdata              : out   std_logic_vector(C_M00_AXIS_TDATA_WIDTH - 1 downto  0);
      m00_axis_tuser              : out   std_logic_vector((C_M00_AXIS_TDATA_WIDTH / 8) - 1 downto  0);
      m00_axis_tready             : in    std_logic
   );
end axis_gtp_bridge_v1_0;

architecture arch_imp of axis_gtp_bridge_v1_0 is

   -----------------------------------------------------------------------------
   -- Parameter
   -----------------------------------------------------------------------------
-- Vivado IP Editor does not work properly with "real" numbers, hence this work-around is/was needed
   constant CPU_CLK_MHz           : real := real(CPU_CLK_Hz) / 1000000.0;     -- Frequency of the stable clock in [MHz]
   constant REF_CLK_MHz           : real := real(REF_CLK_Hz) / 1000000.0;     -- Frequency of the reference clock in [MHz]
   constant BAUD_RATE_Gbps        : real := real(BAUD_RATE_Mbps) / 1000.0;    -- Baudrate in GBit/s
--   constant CPU_CLK_MHz           : real := 125.0;     -- Frequency of the stable clock in [MHz]
--   constant REF_CLK_MHz           : real := 125.0;     -- Frequency of the reference clock in [MHz]
--   constant BAUD_RATE_Gbps        : real := 2.5;       -- Baudrate in GBit/s

   constant PLL_FBDIV             : integer := PARAM_PLL_FBDIV(REF_CLK_MHz, BAUD_RATE_Gbps);
   constant PLL_FBDIV_45          : integer := PARAM_PLL_FBDIV_45(REF_CLK_MHz, BAUD_RATE_Gbps);
   constant PLL_REFCLK_DIV        : integer := PARAM_PLL_REFCLK_DIV(REF_CLK_MHz, BAUD_RATE_Gbps);
   -----------------------------------------------------------------------------
   -- GTP Interface
   -----------------------------------------------------------------------------
   signal   gtrefclk0             : std_logic;
   signal   gtrefclk1             : std_logic;

   signal   gtp_ctrl_in           : gtp_ctrl_in_type;
   signal   gtp_in                : gtp_in_type;
   signal   gtp_out               : gtp_out_type;

   signal   comma_lock_axi        : std_logic := '0';
   signal   comma_lock_axi_s      : std_logic := '0';
   signal   comma_lock_plb        : std_logic := '0';
   signal   comma_lock            : std_logic := '0';

   signal   rx_xoff               : std_logic := '0';
   signal   tx_xoff               : std_logic := '0';

   -- AXIS slave bus interface
   signal   rready                : std_logic;
   signal   rdata                 : std_logic_vector(C_S00_AXIS_TDATA_WIDTH - 1 downto  0);
   signal   ruser                 : std_logic_vector((C_S00_AXIS_TDATA_WIDTH / 8) - 1 downto  0);
   signal   rvalid                : std_logic;

   -- AXIS master bus interface
   signal   tvalid                : std_logic;
   signal   tdata                 : std_logic_vector(C_M00_AXIS_TDATA_WIDTH - 1 downto  0);
   signal   tuser                 : std_logic_vector((C_M00_AXIS_TDATA_WIDTH / 8) - 1 downto  0);
   signal   tready                : std_logic;

   --ML84
   signal i_errcnt_rst : std_logic;
   signal rxdisperr_cnt, rxnotintable_cnt : unsigned(31 downto 0);
   signal pll0lock, pll1lock      : std_logic;

begin

   -----------------------------------------------------------------------------
   -- Debug
   -----------------------------------------------------------------------------
   debug_clk             <= '0';
   debug(31 downto  0) <= std_logic_vector(rxdisperr_cnt);
   debug(63 downto 32) <= std_logic_vector(rxnotintable_cnt);
   debug( 79 downto  64) <= gtp_out.rx.rxdata;
   debug( 81 downto  80) <= gtp_out.rx.rxcharisk;
   debug( 83 downto  82) <= gtp_out.rx.rxdisperr;
   debug( 85 downto  84) <= gtp_out.rx.rxnotintable;
   debug(101 downto  86) <= gtp_in.tx.txdata;
   debug(103 downto 102) <= gtp_in.tx.txcharisk;
   debug(           104) <= pll0lock;
   debug(           105) <= pll1lock;
   debug(           106) <= gtp_out.rx.rxresetdone;
   debug(           107) <= gtp_out.tx.txresetdone;
   debug(           108) <= gtp_out.rx.rxbyteisaligned;
   debug(           109) <= gtp_out.rx.rxbyterealign;
   debug(           110) <= gtp_out.rx.rxcommadet;
   --
   debug(           126) <= gtp_out.rx.rxusrclk;
   debug(           127) <= gtp_out.tx.txusrclk;
--ML84
   i_errcnt_rst <= '0';

   STATS_P : process(gtp_out.rx.rxusrclk)
   begin
       if rising_edge(gtp_out.rx.rxusrclk) then
           if (i_errcnt_rst = '1') then
               rxdisperr_cnt <= (others => '0');
           elsif (gtp_out.rx.rxdisperr /= "00") then
               rxdisperr_cnt <= rxdisperr_cnt + 1;
           end if;

           if (i_errcnt_rst = '1') then
               rxnotintable_cnt <= (others => '0');
           elsif (gtp_out.rx.rxnotintable /= "00") then
               rxnotintable_cnt <= rxnotintable_cnt + 1;
           end if;

       end if;
   end process;

   -----------------------------------------------------------------------------
   -- System
   -----------------------------------------------------------------------------
   gtp_ctrl_in.clk                <= aclk;
   gtp_ctrl_in.rst                <= not aresetn;

   -----------------------------------------------------------------------------
   -- GTP reference clock
   -----------------------------------------------------------------------------
   gtrefclk0                      <= i_clk_gtp( 0);
   gtrefclk1                      <= i_clk_gtp( 1);

   -----------------------------------------------------------------------------
   -- GTP PLL Instance
   -----------------------------------------------------------------------------
   gtp_pll_inst: gtp_pll
   generic map
   (
      --------------------------------------------------------------------------
      -- Simulation
      --------------------------------------------------------------------------
      SIM_RESET_SPEEDUP           => "TRUE",
      SIM_PLL0REFCLK_SEL          => "001",
      SIM_PLL1REFCLK_SEL          => "001",
      --------------------------------------------------------------------------
      -- PLL0
      --------------------------------------------------------------------------
      PLL0_FBDIV                  => PLL_FBDIV,
      PLL0_FBDIV_45               => PLL_FBDIV_45,
      PLL0_REFCLK_DIV             => PLL_REFCLK_DIV,
      --------------------------------------------------------------------------
      -- PLL1
      --------------------------------------------------------------------------
      PLL1_FBDIV                  => PLL_FBDIV,
      PLL1_FBDIV_45               => PLL_FBDIV_45,
      PLL1_REFCLK_DIV             => PLL_REFCLK_DIV
   )

   port map
   (
      --------------------------------------------------------------------------
      -- CPU clock side
      --------------------------------------------------------------------------
      -- System
      CPU_CLK                     => gtp_ctrl_in.clk,
      CPU_RST                     => gtp_ctrl_in.rst,
      --------------------------------------------------------------------------
      -- Reference clock input
      --------------------------------------------------------------------------
      GTREFCLK0                   => gtrefclk0,
      GTREFCLK1                   => gtrefclk1,
      --------------------------------------------------------------------------
      -- PLL0
      --------------------------------------------------------------------------
      -- Reset
      PLL0RESET                   => gtp_out.pll.pll0reset,
      -- Ref clock
      PLL0REFCLKSEL               => "001",
      PLL0REFCLKLOST              => open,
      -- Output clock
      PLL0LOCK                    => pll0lock, --gtp_in.pll.pll0lock,
      PLL0OUTCLK                  => gtp_in.pll.pll0clk,
      PLL0OUTREFCLK               => gtp_in.pll.pll0refclk,
      --------------------------------------------------------------------------
      -- PLL1
      --------------------------------------------------------------------------
      -- Reset
      PLL1RESET                   => gtp_out.pll.pll1reset,
      -- Ref clock
      PLL1REFCLKSEL               => "001",
      PLL1REFCLKLOST              => open,
      -- Output clock
      PLL1LOCK                    => pll1lock, --gtp_in.pll.pll1lock,
      PLL1OUTCLK                  => gtp_in.pll.pll1clk,
      PLL1OUTREFCLK               => gtp_in.pll.pll1refclk
   );
   --ML84
   gtp_in.pll.pll0lock <= pll0lock;
   gtp_in.pll.pll1lock <= pll1lock;

   -----------------------------------------------------------------------------
   -- GTP Rx settings
   -----------------------------------------------------------------------------
   gtp_in.rx.rxpd                 <= "00";

   -----------------------------------------------------------------------------
   -- GTP Tx settings
   -----------------------------------------------------------------------------
   gtp_in.tx.txpd                 <= "00";

   -----------------------------------------------------------------------------
   -- GTP instance
   -----------------------------------------------------------------------------
   gtp_link_inst: gtp_link
   generic map
   (
      --------------------------------------------------------------------------
      -- Simulation attributes
      --------------------------------------------------------------------------
      SIM_RESET_SPEEDUP           => SIM_RESET_SPEEDUP,                       -- Set to "TRUE" to speed up sim reset
      --------------------------------------------------------------------------
      -- Startup timing
      --------------------------------------------------------------------------
      CPU_CLK_MHz                 => CPU_CLK_MHz,                             -- Frequency of the stable clock in [MHz]
      REF_CLK_MHz                 => REF_CLK_MHz,                             -- Frequency of the reference clock in [MHz]
      BAUD_RATE_Gbps              => BAUD_RATE_Gbps                           -- Baudrate in GBit/s
   )
   port map
   (
      i_gtp_ctrl                  => gtp_ctrl_in,
      i_gtp                       => gtp_in,
      o_gtp                       => gtp_out
   );

   -----------------------------------------------------------------------------
   -- RX comma detect
   -----------------------------------------------------------------------------
   rx_comma_detect_inst: rx_comma_detect
   port map
   (
      rxusrclk                    => gtp_out.rx.rxusrclk,
      rxdisperr                   => gtp_out.rx.rxdisperr,
      rxnotintable                => gtp_out.rx.rxnotintable,
      rxcharisk                   => gtp_out.rx.rxcharisk,
      rxdata                      => gtp_out.rx.rxdata,
      --lock info
      comma_lock_axi              => comma_lock_axi,
      comma_lock_plb              => comma_lock_plb,
      comma_lock                  => comma_lock
   );

   o_lock_plb                     <= comma_lock_plb and comma_lock; --ML84 changed
   o_lock_axi                     <= comma_lock_axi and comma_lock; --ML84 added
   o_lock_comma                   <= comma_lock; --ML84 added
   comma_lock_axi_s               <= comma_lock_axi when comma_lock = '1' else --ML84 added
                                     '1'; --Assume AXI while not locked

   -----------------------------------------------------------------------------
   -- RX FIFO
   -----------------------------------------------------------------------------
   rx_gtp_fifo_inst: rx_gtp_fifo
   port map
   (
      --------------------------------------------------------------------------
      -- Debug
      --------------------------------------------------------------------------
      debug_clk                   => open,
      debug                       => open,
      --------------------------------------------------------------------------
      -- GTX clock side
      --------------------------------------------------------------------------
      -- System
      rxusrclk                    => gtp_out.rx.rxusrclk,
      -- GTX rx
      rxresetdone                 => gtp_out.rx.rxresetdone,
      rxdisperr                   => gtp_out.rx.rxdisperr,
      rxnotintable                => gtp_out.rx.rxnotintable,
      rxcharisk                   => gtp_out.rx.rxcharisk,
      rxdata                      => gtp_out.rx.rxdata,
      -- GTX tx flow control
      rx_xoff                     => rx_xoff,
      tx_xoff                     => tx_xoff,
      --------------------------------------------------------------------------
      -- AXI clock side
      --------------------------------------------------------------------------
      -- System
      aclk                        => gtp_ctrl_in.clk,
      -- FIFO status
      axi_tready                  => rready,
      axi_tvalid                  => rvalid,
      axi_tdata                   => rdata,
      axi_tuser                   => ruser
   );

   m00_axis_tvalid                <= rvalid;
   m00_axis_tdata                 <= rdata;
   m00_axis_tuser                 <= ruser;
   rready                         <= m00_axis_tready;

   -----------------------------------------------------------------------------
   -- TX FIFO
   -----------------------------------------------------------------------------
   s00_axis_tready                <= tready;
   tvalid                         <= s00_axis_tvalid;
   tdata                          <= s00_axis_tdata;
   tuser                          <= s00_axis_tuser;

   tx_gtp_fifo_inst: tx_gtp_fifo
   port map
   (
      --------------------------------------------------------------------------
      -- AXI clock side
      --------------------------------------------------------------------------
      -- System
      aclk                        => gtp_ctrl_in.clk,
      -- AXI read interface
      axi_tready                  => tready,
      axi_tvalid                  => tvalid,
      axi_tdata                   => tdata,
      axi_tuser                   => tuser,
      --------------------------------------------------------------------------
      -- GTX clock side
      --------------------------------------------------------------------------
      -- System
      txusrclk                    => gtp_out.tx.txusrclk,
      -- GTX tx flow control
      comma_lock_axi              => comma_lock_axi_s,
      rx_xoff                     => rx_xoff,
      tx_xoff                     => tx_xoff,
      -- GTX rx
      txresetdone                 => gtp_out.tx.txresetdone,
      txcharisk                   => gtp_in.tx.txcharisk,
      txdata                      => gtp_in.tx.txdata
   );

   -----------------------------------------------------------------------------
   -- GTP connections
   -----------------------------------------------------------------------------
   gtp_in.rx.gtprxp               <= i_gtp_rx_p;
   gtp_in.rx.gtprxn               <= i_gtp_rx_n;
   o_gtp_tx_p                     <= gtp_out.tx.gtptxp;
   o_gtp_tx_n                     <= gtp_out.tx.gtptxn;

end arch_imp;
