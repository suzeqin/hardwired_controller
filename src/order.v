// Copyright (c) 2025 Subingbupt
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

module order (
    // 输入端口
    input  wire         SWC,
    input  wire         SWB,
    input  wire         SWA,
    input  wire         oriW3,
    input  wire         oriW2,
    input  wire         oriW1,
    input  wire         CLR,     // 异步复位, 低电平有效
    input  wire         T3,      // 时钟信号
    input  wire [3:0]   IRH,
    input  wire         C,
    input  wire         Z,

    // 输出端口 (全部改为reg类型，因为它们在always块中被赋值)
    output reg          DRW,
    output reg          PCINC,
    output reg          LPC,
    output reg          LAR,
    output reg          PCADD,
    output reg          ARINC,
    output reg          SELCTL,
    output reg          MEMW,
    output reg          LIR,
    output reg          LDZ,
    output reg          LDC,
    output reg          CIN,
    output reg  [3:0]   S,
    output reg          M,
    output reg          ABUS,
    output reg          SBUS,
    output reg          MBUS,
    output reg          SHORT,
    output reg          LONG,
    output reg          SEL0,
    output reg          SEL1,
    output reg          SEL2,
    output reg          SEL3,
    output reg          STOP
);

// --- 宏定义 ---
// cnt 状态机状态定义 (2位版本)
`define STATE_T0 2'b00
`define STATE_T1 2'b01
`define STATE_T2 2'b10

// IRH 操作码定义
`define OP_ADD   4'b0001 // 加法
`define OP_SUB   4'b0010 // 减法
`define OP_AND   4'b0011 // 与
`define OP_INC   4'b0100 // 加一
`define OP_LD    4'b0101 // 从内存加载
`define OP_ST    4'b0110 // 存储到内存
`define OP_JC    4'b0111 // 进位跳转
`define OP_JZ    4'b1000 // 零跳转
`define OP_JMP   4'b1001 // 无条件跳转
`define OP_OUTA  4'b1010 // 输出A
`define OP_NOT   4'b1011 // 非
`define OP_MOV   4'b1100 // 移动
`define OP_OR    4'b1101 // 或
`define OP_STP   4'b1110 // 停止
`define OP_CMP   4'b1111 // 比较

// 内部状态寄存器
reg ST0;
reg [1:0] cnt;

// 内部连线，用于计算所有寄存器的“下一状态”值
wire ST0_next;
wire DRW_next, PCINC_next, LPC_next, LAR_next, PCADD_next, ARINC_next;
wire SELCTL_next, MEMW_next, LIR_next, LDZ_next, LDC_next, CIN_next;
wire [3:0] S_next;
wire M_next, ABUS_next, SBUS_next, MBUS_next, SHORT_next, LONG_next;
wire SEL0_next, SEL1_next, SEL2_next, SEL3_next, STOP_next;
wire SST0;

//----------------------------------------------------------------------
// 1. 组合逻辑部分: 计算所有寄存器的下一状态值
//----------------------------------------------------------------------

// 模式控制码
wire [2:0] SWCBA = {SWC, SWB, SWA};

// 模式解码
wire write_reg_mode = (SWCBA == 3'b100);
wire read_reg_mode  = (SWCBA == 3'b011);
wire read_mem_mode  = (SWCBA == 3'b010);
wire write_mem_mode = (SWCBA == 3'b001);
wire fetch_exec_mode = (SWCBA == 3'b000);
wire W1, W2, W3;

assign W1 = ((write_reg_mode || read_reg_mode) && ((!oriW1 && !oriW2) || oriW2)) ||
            ((read_mem_mode || write_mem_mode) && ((!oriW1 && !oriW2) || oriW1)) ||
            (fetch_exec_mode  && (cnt == `STATE_T2) &&
            (((IRH == `OP_LD || IRH == `OP_ST) && ((!oriW1 && !oriW2 && !oriW3) || oriW3)) ||
            ((IRH != `OP_LD && IRH != `OP_ST) && ((!oriW1 && !oriW2 && !oriW3) || oriW2)))) ||
            (fetch_exec_mode  && (cnt == `STATE_T0 || cnt == `STATE_T1) );

assign W2 = (write_reg_mode || read_reg_mode || (fetch_exec_mode && (cnt == `STATE_T2)) ) && oriW1;

assign W3 = fetch_exec_mode && (cnt == `STATE_T2) && oriW2 && (IRH == `OP_LD || IRH == `OP_ST);
//                             !!!  postpone 1 clock !!!

// SST0的逻辑，用于决定是否进入下一阶段
assign SST0 = (write_reg_mode && !ST0 && W2) ||
              (read_mem_mode && !ST0 && W1) ||
              (write_mem_mode && !ST0 && W1) ||
              (fetch_exec_mode && !ST0 && W1);

// ST0状态寄存器的下一状态逻辑
assign ST0_next = SST0 ? 1'b1 :
                  (write_reg_mode && W2 && ST0) ? 1'b0 :
                  ST0;

// 各输出控制信号的下一状态逻辑
assign DRW_next = (write_reg_mode) ||
                  (fetch_exec_mode && ST0 &&
                   ((IRH == `OP_ADD && W2) ||  // ADD
                    (IRH == `OP_SUB && W2) ||  // SUB
                    (IRH == `OP_AND && W2) ||  // AND
                    (IRH == `OP_INC && W2) ||  // INC
                    (IRH == `OP_LD && W3)  ||  // LD
                    (IRH == `OP_OR && W2)  ||  // OR
                    (IRH == `OP_NOT && W2) ||  // NOT
                    (IRH == `OP_MOV && W2)));  // MOV

assign PCINC_next = (fetch_exec_mode && ST0 && W1);

assign LPC_next = (fetch_exec_mode && !ST0 && W1) ||
                  (fetch_exec_mode && ST0 && IRH == `OP_JMP && W2);  // JMP

assign LAR_next = (read_mem_mode && !ST0 && W1) ||
                  (write_mem_mode && !ST0 && W1) ||
                  (fetch_exec_mode && ST0 &&
                   ((IRH == `OP_LD && W2) ||  // LD
                    (IRH == `OP_ST && W2)));  // ST

assign PCADD_next = (fetch_exec_mode && ST0 &&
                     ((IRH == `OP_JC && C && W2) ||  // JC
                      (IRH == `OP_JZ && Z && W2)));  // JZ

assign ARINC_next = (read_mem_mode && ST0 && W1) ||
                    (write_mem_mode && ST0 && W1);

assign SELCTL_next = write_reg_mode || read_reg_mode ||
                     read_mem_mode || write_mem_mode;

assign MEMW_next = (write_mem_mode && ST0 && W1) ||
                   (fetch_exec_mode && ST0 && IRH == `OP_ST && W3);  // ST

assign LIR_next = (fetch_exec_mode && ST0 && W1);

assign LDZ_next = (fetch_exec_mode && ST0 &&
                   ((IRH == `OP_ADD && W2) ||  // ADD
                    (IRH == `OP_SUB && W2) ||  // SUB
                    (IRH == `OP_AND && W2) ||  // AND
                    (IRH == `OP_INC && W2) ||  // INC
                    (IRH == `OP_CMP && W2) ||  // CMP
                    (IRH == `OP_OR && W2)));  // OR

assign LDC_next = (fetch_exec_mode && ST0 &&
                   ((IRH == `OP_ADD && W2) ||  // ADD
                    (IRH == `OP_SUB && W2) ||  // SUB
                    (IRH == `OP_NOT && W2) ||  // NOT
                    (IRH == `OP_CMP && W2) ||  // CMP
                    (IRH == `OP_INC && W2)));  // INC

assign CIN_next = (fetch_exec_mode && ST0 && IRH == `OP_ADD && W2);  // ADD

assign S_next = (fetch_exec_mode && ST0) ?
                (W1 ? 4'b0000 :
                 (IRH == `OP_ADD) ? 4'b1001 :  // ADD
                 (IRH == `OP_SUB) ? 4'b0110 :  // SUB
                 (IRH == `OP_AND) ? 4'b1011 :  // AND
                 (IRH == `OP_INC) ? 4'b0000 :  // INC
                 (IRH == `OP_LD)  ? 4'b1010 :  // LD
                 (IRH == `OP_ST)  ? (W2 ? 4'b1111 : 4'b1010) :  // ST
                 (IRH == `OP_JMP) ? 4'b1111 :  // JMP
                 (IRH == `OP_OR)  ? 4'b1110 :  // OR
                 (IRH == `OP_OUTA)? 4'b1111 :  // OUT-A
                 (IRH == `OP_MOV) ? 4'b1010 :  // MOV
                 (IRH == `OP_CMP) ? 4'b0110 :  // CMP
                 (IRH == `OP_NOT) ? 4'b0000 :  // NOT
                 4'b0000) : 4'b0000;

assign M_next = (fetch_exec_mode && ST0 &&
                 ((IRH == `OP_AND && W2) ||  // AND
                  (IRH == `OP_LD && W2)  ||  // LD
                  (IRH == `OP_ST && (W2 || W3)) ||  // ST
                  (IRH == `OP_JMP && W2) ||  // JMP
                  (IRH == `OP_OR && W2)  ||  // OR
                  (IRH == `OP_OUTA && W2)||  // OUT-A
                  (IRH == `OP_NOT && W2) ||  // NOT
                  (IRH == `OP_MOV && W2)));  // MOV

assign ABUS_next = (fetch_exec_mode && ST0 &&
                    ((IRH == `OP_ADD && W2) ||  // ADD
                     (IRH == `OP_SUB && W2) ||  // SUB
                     (IRH == `OP_AND && W2) ||  // AND
                     (IRH == `OP_INC && W2) ||  // INC
                     (IRH == `OP_LD && W2)  ||  // LD
                     (IRH == `OP_ST && (W2 || W3)) ||  // ST
                     (IRH == `OP_JMP && W2) ||  // JMP
                     (IRH == `OP_OR && W2)  ||  // OR
                     (IRH == `OP_OUTA && W2)||  // OUT-A
                     (IRH == `OP_NOT && W2) ||  // NOT
                     (IRH == `OP_CMP && W2) ||  // CMP
                     (IRH == `OP_MOV && W2)));  // MOV

assign SBUS_next = (write_reg_mode) ||
                   (read_mem_mode && !ST0 && W1) ||
                   (write_mem_mode && W1) ||
                   (fetch_exec_mode && !ST0 && W1);

assign MBUS_next = (read_mem_mode && ST0 && W1) ||
                   (fetch_exec_mode && ST0 && IRH == `OP_LD && W3);  // LD

assign SHORT_next = (read_mem_mode) || (write_mem_mode) ||
                    (fetch_exec_mode && !ST0 && W1);

assign LONG_next = (fetch_exec_mode && ST0 &&
                    ((IRH == `OP_LD && W2) ||  // LD
                     (IRH == `OP_ST && W2)));  // ST

assign SEL0_next = (write_reg_mode && W1) ||
                   (read_reg_mode);

assign SEL1_next = (write_reg_mode && ((!ST0 && W1) || (ST0 && W2))) ||
                   (read_reg_mode && W2);

assign SEL2_next = (write_reg_mode && W2);

assign SEL3_next = (write_reg_mode && ST0) ||
                   (read_reg_mode && W2);

assign STOP_next = write_reg_mode || read_reg_mode ||
                   read_mem_mode || write_mem_mode ||
                   (fetch_exec_mode && !ST0) ||
                   (fetch_exec_mode && ST0 && IRH == `OP_STP);  // STP


//----------------------------------------------------------------------
// 2. 时序逻辑部分: 在时钟沿更新所有寄存器
//----------------------------------------------------------------------
always @(negedge T3 or negedge CLR) begin
    if (!CLR) begin
        // 异步复位：所有寄存器清零
        ST0    <= 1'b0;
        DRW    <= 1'b0;
        PCINC  <= 1'b0;
        LPC    <= 1'b0;
        LAR    <= 1'b0;
        PCADD  <= 1'b0;
        ARINC  <= 1'b0;
        SELCTL <= 1'b0;
        MEMW   <= 1'b0;
        LIR    <= 1'b0;
        LDZ    <= 1'b0;
        LDC    <= 1'b0;
        CIN    <= 1'b0;
        S      <= 4'b0;
        M      <= 1'b0;
        ABUS   <= 1'b0;
        SBUS   <= 1'b0;
        MBUS   <= 1'b0;
        SHORT  <= 1'b0;
        LONG   <= 1'b0;
        SEL0   <= 1'b0;
        SEL1   <= 1'b0;
        SEL2   <= 1'b0;
        SEL3   <= 1'b0;
        STOP   <= 1'b0;
        cnt    <= `STATE_T0;
    end else begin
        if (cnt == `STATE_T0 || cnt == `STATE_T1) begin
            cnt <= cnt + 2'b01;
        end else begin
            //
        end
        // 时钟有效沿：用计算好的下一状态值更新寄存器
        ST0    <= ST0_next;
        DRW    <= DRW_next;
        PCINC  <= PCINC_next;
        LPC    <= LPC_next;
        LAR    <= LAR_next;
        PCADD  <= PCADD_next;
        ARINC  <= ARINC_next;
        SELCTL <= SELCTL_next;
        MEMW   <= MEMW_next;
        LIR    <= LIR_next;
        LDZ    <= LDZ_next;
        LDC    <= LDC_next;
        CIN    <= CIN_next;
        S      <= S_next;
        M      <= M_next;
        ABUS   <= ABUS_next;
        SBUS   <= SBUS_next;
        MBUS   <= MBUS_next;
        SHORT  <= SHORT_next;
        LONG   <= LONG_next;
        SEL0   <= SEL0_next;
        SEL1   <= SEL1_next;
        SEL2   <= SEL2_next;
        SEL3   <= SEL3_next;
        STOP   <= STOP_next;
    end
end

endmodule