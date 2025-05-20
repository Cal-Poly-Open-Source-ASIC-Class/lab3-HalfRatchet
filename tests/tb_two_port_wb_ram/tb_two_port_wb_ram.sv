`timescale 1ns/1ps
`default_nettype none

module tb_two_port_wb_ram;
    `ifdef USE_POWER_PINS
    wire VPWR;
    wire VGND;
    assign VPWR = 1;
    assign VGND = 0;
    `endif

    localparam ADDR_WIDTH = 11;
    localparam DATA_WIDTH = 32;
    localparam SEL_WIDTH  = 4;

    logic clk, rst;

    // Wishbone Port A
    logic [ADDR_WIDTH-1:0] pA_wb_addr_i;
    logic [DATA_WIDTH-1:0] pA_wb_data_i;
    logic [SEL_WIDTH-1:0]  pA_wb_sel_i;
    logic pA_wb_we_i;
    logic pA_wb_cyc_i;
    logic pA_wb_stb_i;
    logic pA_wb_ack_o;
    logic [DATA_WIDTH-1:0] pA_wb_data_o;
    logic pA_wb_stall_o;

    // Wishbone Port B
    logic [ADDR_WIDTH-1:0] pB_wb_addr_i;
    logic [DATA_WIDTH-1:0] pB_wb_data_i;
    logic [SEL_WIDTH-1:0]  pB_wb_sel_i;
    logic pB_wb_we_i;
    logic pB_wb_cyc_i;
    logic pB_wb_stb_i;
    logic pB_wb_ack_o;
    logic [DATA_WIDTH-1:0] pB_wb_data_o;
    logic pB_wb_stall_o;

    // Instantiate DUT
    two_port_wb_ram dut (
        .clk(clk), .rst(rst),
        .pA_wb_addr_i, .pA_wb_data_i, .pA_wb_sel_i,
        .pA_wb_we_i, .pA_wb_cyc_i, .pA_wb_stb_i,
        .pA_wb_ack_o, .pA_wb_data_o, .pA_wb_stall_o,
        .pB_wb_addr_i, .pB_wb_data_i, .pB_wb_sel_i,
        .pB_wb_we_i, .pB_wb_cyc_i, .pB_wb_stb_i,
        .pB_wb_ack_o, .pB_wb_data_o, .pB_wb_stall_o, .VPWR(VPWR),
        .VGND(VGND)
    );


    always #5 clk = ~clk;
    initial begin
        clk = 0;
        rst = 1;
        // Initialize all inputs to avoid X states
        pA_wb_addr_i = 0;
        pA_wb_data_i = 0;
        pA_wb_sel_i = 0;
        pA_wb_we_i = 0;
        pA_wb_cyc_i = 0;
        pA_wb_stb_i = 0;
        pB_wb_addr_i = 0;
        pB_wb_data_i = 0;
        pB_wb_sel_i = 0;
        pB_wb_we_i = 0;
        pB_wb_cyc_i = 0;
        pB_wb_stb_i = 0;
        #20 rst = 0;
    end

    // Write Task 
    task automatic wb_write(input bit isA, input [ADDR_WIDTH-1:0] addr, input [DATA_WIDTH-1:0] data);
        if (isA) begin
            @(posedge clk);
            pA_wb_addr_i = addr;
            pA_wb_data_i = data;
            pA_wb_sel_i  = 4'b1111;
            pA_wb_we_i   = 1;
            pA_wb_cyc_i  = 1;
            pA_wb_stb_i  = 1;
            
            do begin
                @(posedge clk);
            end while (!pA_wb_ack_o || pA_wb_stall_o);
            
            pA_wb_cyc_i = 0;
            pA_wb_stb_i = 0;
            pA_wb_we_i = 0;
        end else begin
            @(posedge clk);
            pB_wb_addr_i = addr;
            pB_wb_data_i = data;
            pB_wb_sel_i  = 4'b1111;
            pB_wb_we_i   = 1;
            pB_wb_cyc_i  = 1;
            pB_wb_stb_i  = 1;
            
            do begin
                @(posedge clk);
            end while (!pB_wb_ack_o || pB_wb_stall_o);
            
            pB_wb_cyc_i = 0;
            pB_wb_stb_i = 0;
            pB_wb_we_i = 0;
        end
    endtask

    // Read Task 
    task automatic wb_read(input bit isA, input [ADDR_WIDTH-1:0] addr, output [DATA_WIDTH-1:0] out);
        if (isA) begin
            @(posedge clk);
            pA_wb_addr_i = addr;
            pA_wb_sel_i  = 4'b1111;
            pA_wb_we_i   = 0;
            pA_wb_cyc_i  = 1;
            pA_wb_stb_i  = 1;
            

            do begin
                @(posedge clk);
            end while (!pA_wb_ack_o || pA_wb_stall_o);
            
            @(posedge clk);
            out = pA_wb_data_o;
            
            pA_wb_cyc_i = 0;
            pA_wb_stb_i = 0;
        end else begin
            @(posedge clk);
            pB_wb_addr_i = addr;
            pB_wb_sel_i  = 4'b1111;
            pB_wb_we_i   = 0;
            pB_wb_cyc_i  = 1;
            pB_wb_stb_i  = 1;
            
            do begin
                @(posedge clk);
            end while (!pB_wb_ack_o || pB_wb_stall_o);
            
            @(posedge clk);
            out = pB_wb_data_o;
            
            pB_wb_cyc_i = 0;
            pB_wb_stb_i = 0;
        end
    endtask
    
    logic [31:0] valA, valB;

    initial begin
        wait (!rst);
        $display("Test: No Conflict (different macros)");

        wb_write(1'b0, 11'b000_0000_0100, 32'hA5A5A5A5);  // Port B → RAM0, addr 1
        wb_write(1'b1, 11'b100_0000_0100, 32'h5A5A5A5A);  // Port A → RAM1, addr 1

        wb_read(1'b0, 11'b000_0000_0100, valB);
        wb_read(1'b1, 11'b100_0000_0100, valA);

        $display("Read B = %h (expected A5A5A5A5)", valB);
        $display("Read A = %h (expected 5A5A5A5A)", valA);

        #20;

        fork
            wb_write(1'b0, 11'b000_0000_1000, 32'h12345678);  // Port B -> RAM0, addr 2
            wb_write(1'b1, 11'b000_0000_1100, 32'h87654321);  // Port A -> RAM0, addr 3
        join

        wb_read(1'b0, 11'b000_0000_1000, valB);
        wb_read(1'b1, 11'b000_0000_1100, valA);

        $display("Read B = %h (expect 12345678)", valB);
        #100
        $display("Read A = %h (expect 87654321)", valA);

        $display("Test complete");
        $finish;
    end

    initial begin
        $dumpfile("two_port_wb_ram_tb.vcd");
        $dumpvars(2, tb_two_port_wb_ram);
    end

endmodule