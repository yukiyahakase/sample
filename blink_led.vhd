----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date:    23:21:15 01/27/2016 
-- Design Name: 
-- Module Name:    blink_led - Behavioral 
-- Project Name: 
-- Target Devices: 
-- Tool versions: 
-- Description: 
--
-- Dependencies: 
--
-- Revision: 
-- Revision 0.01 - File Created
-- Additional Comments: 
--
----------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx primitives in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity blink_led is
    Port ( clk : in  STD_LOGIC;
           resetb : in  STD_LOGIC;
           led : out  STD_LOGIC);
end blink_led;

architecture Behavioral of blink_led is

signal cnt : std_logic_vector(23 downto 0) := "000000000000000000000000";

begin
  process(clk, resetb)
  begin
    if (resetb = '1') then
	   cnt <= "000000000000000000000000";
    else
      if (clk'event and clk = '1') then
	     cnt <= cnt + "000000000000000000000001";
		end if;
	 end if;
  end process;

  led <= cnt(23);

end Behavioral;
