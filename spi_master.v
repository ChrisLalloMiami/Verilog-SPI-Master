`timescale 1ns / 1ps

// Implementation of the SPI protocol, intended for use with the Zybo Z7-10. 
// Note: Clocking wizard pulls system clock down to 100 MHz for locking with other slaves.


module spi_master (
    input clk, rst,
    input miso,
    input enable,
    output reg mosi, sclk, ss, done
);

// Generate common 100 MHz clock
clk_wiz_0 clock_wizard (
    .clk_in1  (clk),
    .resetn   (rst),
    .clk_out1 (new_clk),
    .locked   (clk_lock)
);

// Temporary test values
localparam [15:0]message   = 16'b1100110110101010;
localparam [1:0]numBytes   = 2'd2;

//wire miso;
reg misoOut;

reg [3:0]bitCounter;     // References the current bit
reg [2:0]readBitCounter; // References the current bit of the slave reply
reg [5:0]counter;        // General counter for waiting


// States for state machine
reg [3:0]state;
parameter waiting      = 4'd0,
          idle         = 4'd1,
          startHigh    = 4'd2,
          startLow     = 4'd3,
          lastHigh     = 4'd4,
          lastLow      = 4'd5,
          listenHigh   = 4'd6,
          listenLow    = 4'd7,
          readLastHigh = 4'd8,
          readLastLow  = 4'd9,
          finish       = 4'd10;

// Handles all logic for state transitions
always @(posedge new_clk or negedge rst) begin
    if (rst == 1'b0)
        state <= idle;
    else
    case (state)
        // Waits for enable signal to go high before starting transmission. Sets first data bit of MOSI
        waiting:
            if (enable == 1'b1)
                state <= idle;
            else
                state <= waiting;
    
        // Initial wait state
        idle:
            if (counter == 47)
                state <= startHigh;
            else
                state <= idle;
        
        // Raises SCLK
        startHigh:
            if (counter == 49)
                state <= startLow;
            else
                state <= startHigh;
                
        // Lowers SCLK and decrements bitCounter
        startLow:
            if (bitCounter == 0)
                state <= lastHigh;
            else
                if (counter == 49)
                    state <= startHigh;
                else
                    state <= startLow;
        
        // Raises the clock once more after final MOSI write for slave to read
        lastHigh:
            if (counter == 50)
                state <= lastLow;
            else
                state <= lastHigh;
        
        // Lowers the clock once more after final MOSI write for slave to read
        lastLow:
            if (counter == 49)
                if (message[8] == 1'b1) // Check for W/R bit
                    state <= listenHigh;
                else
                    state <= finish;
            else
                state <= lastLow;
                
        // Holds the clock low for half a cycle to listen for slave reply
        listenHigh:
            if (readBitCounter == 0)
                state <= readLastHigh;
            else
                if (counter == 49)
                    state <= listenLow;
                else
                    state <= listenHigh;
                
        // Holds the clock low for half a cycle to listen for slave reply
        listenLow:
            if (counter == 49)
                state <= listenHigh;
            else
                state <= listenLow;
        
        // Holds the clock low for one more cycle after reading for final bit
        readLastHigh:
            if (counter == 49)
                state <= readLastLow;
            else
                state <= readLastHigh;
        
        // Holds the clock high for one more cycle after reading for final bit
        readLastLow:
            if (counter == 49)
                state <= finish;
            else
                state <= readLastLow;
        
        // Ending state
        finish: state <= finish;
                
        // Default case to avoid latch
        default: begin end
    endcase     
end

// Handles all logic within each state
always @(posedge new_clk or negedge rst) begin
    if (rst == 1'b0) begin
        bitCounter     <= (numBytes * 8) - 1;
        readBitCounter <= 3'd7;
        counter        <= 6'd0;
        
        mosi           <= 1'b0;
        sclk           <= 1'b1;
        ss             <= 1'b1;
        done           <= 1'b0;
        misoOut        <= 1'b0;
    end
    else
    case (state)
        // Waits for enable signal to go high before starting transmission. Sets first data bit of MOSI
        waiting: begin
            bitCounter     <= (numBytes * 8) - 1;
            readBitCounter <= 3'd7;
            counter        <= 6'd0;
            
            mosi           <= message[bitCounter]; // Set MOSI to initial data bit
            sclk           <= 1'b1;
            ss             <= 1'b1;
            done           <= 1'b0;
            misoOut        <= 1'b0;
        end
        
        // Initial wait state
        idle: begin
            if (counter < 47) // Wait half a cycle before starting transmission (minus state transition)
                counter <= counter + 1'd1;
            else
                counter <= 6'd0;
        end
        
        // Raises SCLK
        startHigh: begin
            sclk <= 1'b1;
            ss <= 1'b0;
            
            if (counter < 49) // Hold SCLK high half a cycle
                counter <= counter + 1'd1;
            else
                counter <= 6'd0;
        end
        
        // Lowers SCLK and decrements bitCounter
        startLow: begin
            sclk <= 1'b0;
            mosi <= message[bitCounter]; // Set MOSI to next bit
            
            if (counter < 49) // Hold SCLK high half a cycle
                counter <= counter + 1'd1;
            else begin
                counter <= 6'd0;
                bitCounter <= bitCounter - 1'd1; // Decrement bitCounter for next write
            end
        end
        
        // Raises the clock once more after final MOSI write for slave to read
        lastHigh:
            if (counter < 50)
                counter <= counter + 1'd1;
            else begin
                sclk <= 1'b1;
                counter <= 6'd0;
            end
                
        // Lowers the clock once more after final MOSI write for slave to read
        lastLow:
            if (counter < 49)
                counter <= counter + 1'd1;
            else begin
                sclk <= 1'b0;
                counter <= 6'd0;
            end
            
        // Holds the clock low for half a cycle to listen for slave reply
        listenHigh:
            if (counter < 49)
                counter <= counter + 1'd1;
            else begin
                misoOut <= miso;
                sclk <= 1'b1;
                counter <= 6'd0;
            end
            
        // Holds the clock low for half a cycle to listen for slave reply
        listenLow:
            if (counter < 49)
                counter <= counter + 1'd1;
            else begin
                sclk <= 1'b0;
                counter <= 6'd0;
                readBitCounter <= readBitCounter - 1'd1;
            end
            
        // Holds the clock low for one more cycle after reading for final bit
        readLastHigh:
            if (counter < 49)
                counter <= counter + 1'd1;
            else begin
                sclk <= 1'b1;
                counter <= 6'd0;
            end
            
        // Holds the clock High for one more cycle after reading for final bit
        readLastLow:
            if (counter < 49)
                counter <= counter + 1'd1;
            else begin
                sclk <= 1'b0;
                counter <= 6'd0;
            end
            
        // Ending state
        finish: begin
            ss <= 1'b1;
            done <= 1'b1;
        end
        // Default case to avoid latch
        default: begin end
    endcase
end

endmodule
