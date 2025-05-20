`timescale 1ns/1ps
`default_nettype none

module two_port_wb_ram #(
    parameter ADDR_WIDTH = 11,  
    parameter DATA_WIDTH = 32,
    parameter SEL_WIDTH = 4
)(
    input  logic                 clk,
    input  logic                 rst,

    // Wishbone Port A
    input  logic [ADDR_WIDTH-1:0] pA_wb_addr_i,
    input  logic [DATA_WIDTH-1:0] pA_wb_data_i,
    input  logic [SEL_WIDTH-1:0]  pA_wb_sel_i,
    input  logic pA_wb_we_i,
    input  logic pA_wb_cyc_i,
    input  logic pA_wb_stb_i,
    output logic pA_wb_ack_o,
    output logic [DATA_WIDTH-1:0] pA_wb_data_o,
    output logic  pA_wb_stall_o,

    // Wishbone Port B
    input  logic [ADDR_WIDTH-1:0] pB_wb_addr_i,
    input  logic [DATA_WIDTH-1:0] pB_wb_data_i,
    input  logic [SEL_WIDTH-1:0]  pB_wb_sel_i,
    input  logic pB_wb_we_i,
    input  logic pB_wb_cyc_i,
    input  logic pB_wb_stb_i,
    output logic pB_wb_ack_o,
    output logic [DATA_WIDTH-1:0] pB_wb_data_o,
    output logic pB_wb_stall_o
);

    // Memory select 
    logic memA_sel, memB_sel;  // 0 = ram0, 1 = ram1
    assign memA_sel = pA_wb_addr_i[ADDR_WIDTH-1]; 
    assign memB_sel = pB_wb_addr_i[ADDR_WIDTH-1];

    // Addresses
    logic [7:0] ramA_addr, ramB_addr;
    assign ramA_addr = pA_wb_addr_i[9:2]; 
    assign ramB_addr = pB_wb_addr_i[9:2]; 

    // Valid port 
    logic portA_valid, portB_valid;
    assign portA_valid = pA_wb_cyc_i && pA_wb_stb_i;
    assign portB_valid = pB_wb_cyc_i && pB_wb_stb_i;

    // Conflict detection 
    logic conflict;
    assign conflict = (memA_sel == memB_sel) && portA_valid && portB_valid;

    // Port priority 
    logic portA_priority, portA_priority_next;
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            portA_priority <= 1'b0;
        end else if (conflict) begin
            portA_priority_next <= ~portA_priority_next;
            portA_priority <= portA_priority;
        end
        else
            portA_priority <= portA_priority_next;
    end

    //prirorirty_next gets updated, and that gets updated every next cycle
    // Port stall and ack signals
    assign pA_wb_stall_o = conflict && !portA_priority;
    assign pB_wb_stall_o = conflict && portA_priority;
    
    // Ack
    assign pA_wb_ack_o = portA_valid && !pA_wb_stall_o;
    assign pB_wb_ack_o = portB_valid && !pB_wb_stall_o;
    

    logic [7:0] ram0_addr, ram1_addr;
    logic [31:0] ram0_wdata, ram1_wdata;
    logic [31:0] ram0_rdata, ram1_rdata;
    logic [3:0] ram0_we, ram1_we;
    logic ram0_en, ram1_en;

    // Port A output register
    always_ff @(posedge clk) begin
        if (portA_valid && !pA_wb_stall_o && !pA_wb_we_i) begin
            if (memA_sel == 1'b0) begin
                pA_wb_data_o <= ram0_rdata;
            end else begin
                pA_wb_data_o <= ram1_rdata;
            end
        end
        else
            pA_wb_data_o <= 0;
    end
    
    // Port B output register
    always_ff @(posedge clk) begin
        if (portB_valid && !pB_wb_stall_o && !pB_wb_we_i) begin
            if (memB_sel == 1'b0) begin
                pB_wb_data_o <= ram0_rdata;
            end else begin
                pB_wb_data_o <= ram1_rdata;
            end
        end
        else
            pB_wb_data_o <= 0;
    end

    // RAM access 
    always_comb begin
        // Default 
        ram0_addr = 8'h00;
        ram0_wdata = 32'h00000000;
        ram0_we = 4'b0000;
        ram0_en = 1'b0;
        
        ram1_addr = 8'h00;
        ram1_wdata = 32'h00000000;
        ram1_we = 4'b0000;
        ram1_en = 1'b0;

        // Port A access to RAM0
        if (portA_valid && !pA_wb_stall_o && memA_sel == 1'b0) begin
            ram0_addr = ramA_addr;
            ram0_wdata = pA_wb_data_i;
            ram0_we = pA_wb_we_i ? pA_wb_sel_i : 4'b0000;
            ram0_en = 1'b1;
        end
        
        // Port A access to RAM1
        else if (portA_valid && !pA_wb_stall_o && memA_sel == 1'b1) begin
            ram1_addr = ramA_addr;
            ram1_wdata = pA_wb_data_i;
            ram1_we = pA_wb_we_i ? pA_wb_sel_i : 4'b0000;
            ram1_en = 1'b1;
        end
        
        // Port B access to RAM0
        if (portB_valid && !pB_wb_stall_o && memB_sel == 1'b0) begin
            ram0_addr = ramB_addr;
            ram0_wdata = pB_wb_data_i;
            ram0_we = pB_wb_we_i ? pB_wb_sel_i : 4'b0000;
            ram0_en = 1'b1;
        end
        
        // Port B access to RAM1
        else if (portB_valid && !pB_wb_stall_o && memB_sel == 1'b1) begin
            ram1_addr = ramB_addr;
            ram1_wdata = pB_wb_data_i;
            ram1_we = pB_wb_we_i ? pB_wb_sel_i : 4'b0000;
            ram1_en = 1'b1;
        end
    end

    DFFRAM256x32 ram0 (
        .CLK(clk),
        .EN0(ram0_en),
        .WE0(ram0_we),
        .A0(ram0_addr),
        .Di0(ram0_wdata),
        .Do0(ram0_rdata)
    );

    DFFRAM256x32 ram1 (
        .CLK(clk),
        .EN0(ram1_en),
        .WE0(ram1_we),
        .A0(ram1_addr),
        .Di0(ram1_wdata),
        .Do0(ram1_rdata)
    );

endmodule