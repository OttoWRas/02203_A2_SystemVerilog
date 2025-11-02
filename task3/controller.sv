// -----------------------------------------------------------------------------
//
//  Title      :  Controller to manange the picture transfer to and from the PC.
//             :
//  Developers :  Otto Westy Rasmussen - s203838@dtu.dk
//             :
//  Purpose    :  Controller to manage the picture transfer to and from the PC.
//             :
//  Revision   :  02203 fall 2025 v.1.0
//
// -----------------------------------------------------------------------------

module controller #(
    parameter MEMORY_ADDR_SIZE = 16
) (
    input logic clk,
    input logic reset,

    output logic [7:0] data_stream_tx,
    output logic       data_stream_tx_stb,
    input  logic       data_stream_tx_ack,
    input  logic [7:0] data_stream_rx,
    input  logic       data_stream_rx_stb,

    output logic                        mem_en,
    output logic                        mem_we,
    output logic [MEMORY_ADDR_SIZE-1:0] mem_addr,
    output logic [                31:0] mem_dw,
    input  logic [                31:0] mem_dr
);

  // Address constants
  localparam [MEMORY_ADDR_SIZE-1:0] ADDR_COUNT_MIN = '0;
  localparam [MEMORY_ADDR_SIZE-1:0] ADDR_COUNT_DOWNLOAD_START = '0;
  localparam [MEMORY_ADDR_SIZE-1:0] ADDR_COUNT_DOWNLOAD_END = 25343;
  localparam [MEMORY_ADDR_SIZE-1:0] ADDR_COUNT_UPLOAD_START = 25344;
  localparam [MEMORY_ADDR_SIZE-1:0] ADDR_COUNT_UPLOAD_END = 50687;
  localparam [MEMORY_ADDR_SIZE-1:0] ADDR_COUNT_MAX = 65535;

  typedef enum logic [4:0] {
    START,
    WAIT_AND_CHECK_COMMAND,
    REPLY_TEST,
    CLEAR,
    DOWNLOAD_B0,
    DOWNLOAD_B1,
    DOWNLOAD_B2,
    DOWNLOAD_B3,
    STORE_DOWNLOAD,
    UPLOAD_B0,
    UPLOAD_B1,
    UPLOAD_B2,
    UPLOAD_B3,
    UPLOAD_WAIT,
    UPLOAD_WAIT2,
    UPLOAD_CHECK
  } state_type;

  state_type state, state_next;

  logic [31:0] data_down_buffer, data_down_buffer_next;
  logic [31:0] data_up_buffer, data_up_buffer_next;
  logic [MEMORY_ADDR_SIZE-1:0] addr_count, addr_count_next;
  logic mem_en_next, mem_we_next;

  assign mem_addr = addr_count;
  assign mem_dw   = data_down_buffer;

  always_comb begin
    data_stream_tx        = 8'b0;
    data_stream_tx_stb    = 1'b0;
    state_next            = state;
    data_down_buffer_next = data_down_buffer;
    data_up_buffer_next   = data_up_buffer;
    addr_count_next       = addr_count;
    mem_en_next           = 1'b0;
    mem_we_next           = 1'b0;

    case (state)
      START: begin
        data_down_buffer_next = 32'b0;
        addr_count_next  = ADDR_COUNT_MIN;
        state_next       = CLEAR;
        mem_en_next      = 1'b1;
        mem_we_next      = 1'b1;
      end
      CLEAR: begin
        if (addr_count == ADDR_COUNT_MAX) state_next = WAIT_AND_CHECK_COMMAND;
        else begin
          addr_count_next = addr_count + 1;
          state_next      = CLEAR;
          mem_en_next     = 1'b1;
          mem_we_next     = 1'b1;
        end
      end
      WAIT_AND_CHECK_COMMAND: begin
        if (!data_stream_rx_stb) state_next = WAIT_AND_CHECK_COMMAND;
        else begin
          if (data_stream_rx == 8'h74)  // 't'
            state_next = REPLY_TEST;
          else if (data_stream_rx == 8'h72) begin  // 'r'
            addr_count_next = ADDR_COUNT_UPLOAD_START;
            mem_en_next     = 1'b1;
            state_next      = UPLOAD_WAIT;
          end else if (data_stream_rx == 8'h77) begin  // 'w'
            addr_count_next = ADDR_COUNT_DOWNLOAD_START;
            state_next      = DOWNLOAD_B0;
          end else if (data_stream_rx == 8'h63) begin  // 'c'
            data_down_buffer_next = 32'b0;
            addr_count_next       = ADDR_COUNT_MIN;
            mem_en_next           = 1'b1;
            mem_we_next           = 1'b1;
            state_next            = CLEAR;
          end else state_next = WAIT_AND_CHECK_COMMAND;
        end
      end
      REPLY_TEST: begin
        data_stream_tx     = 8'h79;  // 'y'
        data_stream_tx_stb = 1'b1;
        if (!data_stream_tx_ack) state_next = REPLY_TEST;
        else state_next = WAIT_AND_CHECK_COMMAND;
      end
      DOWNLOAD_B0: begin
        if (!data_stream_rx_stb) state_next = DOWNLOAD_B0;
        else begin
          data_down_buffer_next[7:0] = data_stream_rx;
          state_next                 = DOWNLOAD_B1;
        end
      end
      DOWNLOAD_B1: begin
        if (!data_stream_rx_stb) state_next = DOWNLOAD_B1;
        else begin
          data_down_buffer_next[15:8] = data_stream_rx;
          state_next                  = DOWNLOAD_B2;
        end
      end
      DOWNLOAD_B2: begin
        if (!data_stream_rx_stb) state_next = DOWNLOAD_B2;
        else begin
          data_down_buffer_next[23:16] = data_stream_rx;
          state_next                   = DOWNLOAD_B3;
        end
      end
      DOWNLOAD_B3: begin
        if (!data_stream_rx_stb) state_next = DOWNLOAD_B3;
        else begin
          data_down_buffer_next[31:24] = data_stream_rx;
          state_next                   = STORE_DOWNLOAD;
          mem_en_next                  = 1'b1;
          mem_we_next                  = 1'b1;
        end
      end
      STORE_DOWNLOAD: begin
        if (addr_count == ADDR_COUNT_DOWNLOAD_END) state_next = WAIT_AND_CHECK_COMMAND;
        else begin
          addr_count_next = addr_count + 1;
          state_next      = DOWNLOAD_B0;
        end
      end
      UPLOAD_WAIT: begin
        state_next = UPLOAD_WAIT2;
      end
      UPLOAD_WAIT2: begin
        state_next          = UPLOAD_B0;
        data_up_buffer_next = mem_dr;
      end
      UPLOAD_B0: begin
        data_stream_tx     = data_up_buffer[7:0];
        data_stream_tx_stb = 1'b1;
        if (!data_stream_tx_ack) state_next = UPLOAD_B0;
        else state_next = UPLOAD_B1;
      end
      UPLOAD_B1: begin
        data_stream_tx     = data_up_buffer[15:8];
        data_stream_tx_stb = 1'b1;
        if (!data_stream_tx_ack) state_next = UPLOAD_B1;
        else state_next = UPLOAD_B2;
      end
      UPLOAD_B2: begin
        data_stream_tx     = data_up_buffer[23:16];
        data_stream_tx_stb = 1'b1;
        if (!data_stream_tx_ack) state_next = UPLOAD_B2;
        else state_next = UPLOAD_B3;
      end
      UPLOAD_B3: begin
        data_stream_tx     = data_up_buffer[31:24];
        data_stream_tx_stb = 1'b1;
        if (!data_stream_tx_ack) begin
          state_next = UPLOAD_B3;
        end else state_next = UPLOAD_CHECK;
      end
      UPLOAD_CHECK: begin
        if (addr_count == ADDR_COUNT_UPLOAD_END) state_next = WAIT_AND_CHECK_COMMAND;
        else begin
          addr_count_next = addr_count + 1;
          state_next      = UPLOAD_WAIT;
          mem_en_next     = 1'b1;
        end
      end
      default: state_next = state;
    endcase
  end

  always_ff @(posedge clk) begin
    if (reset) begin
      state            <= START;
      data_down_buffer <= 32'b0;
      data_up_buffer   <= 32'b0;
      addr_count       <= '0;
      mem_en           <= 1'b0;
      mem_we           <= 1'b0;
    end else begin
      state            <= state_next;
      data_down_buffer <= data_down_buffer_next;
      data_up_buffer   <= data_up_buffer_next;
      addr_count       <= addr_count_next;
      mem_en           <= mem_en_next;
      mem_we           <= mem_we_next;
    end
  end

endmodule
