// -----------------------------------------------------------------------------
//
//  Title      :  Implements a universal asynchronous receiver transmitter (UART).
//             :
//  Developers :  Otto Westy Rasmussen - s203838@dtu.dk
//             :
//  Purpose    :  Controller to manage the picture transfer to and from the PC.
//             :
//  Revision   :  02203 fall 2025 v.1.0
//
// -----------------------------------------------------------------------------

module uart #(
    parameter P_BAUD_RATE       = 115200,
    parameter P_CLOCK_FREQUENCY = 100_000_000
) (
    input logic clk,
    input logic rst,
    input logic rx,
    output logic tx,
    input logic [7:0] data_stream_in,
    input logic data_stream_in_stb,
    output logic data_stream_in_ack,
    output logic [7:0] data_stream_out,
    output logic data_stream_out_stb
);

  // RX Clock divider
  localparam integer RX_CLKS_PER_BIT = P_CLOCK_FREQUENCY / (P_BAUD_RATE * 16);
  logic [$clog2(RX_CLKS_PER_BIT)-1:0] rx_tick_counter;
  logic rx_tick;

  always_ff @(posedge clk or posedge rst) begin
    if (rst) begin
      rx_tick_counter <= '0;
      rx_tick         <= 1'b0;
    end else if (rx_tick_counter == RX_CLKS_PER_BIT - 1) begin
      rx_tick_counter <= '0;
      rx_tick         <= 1'b1;
    end else begin
      rx_tick_counter <= rx_tick_counter + 1;
      rx_tick         <= 1'b0;
    end
  end

  // RX Synchronizer
  logic rx_sync_0, rx_sync_1;
  always_ff @(posedge clk or posedge rst) begin
    if (rst) begin
      rx_sync_0 <= 1'b1;
      rx_sync_1 <= 1'b1;
    end else begin
      rx_sync_0 <= rx;
      rx_sync_1 <= rx_sync_0;
    end
  end

  // RX Over-sampling
  logic [3:0] rx_sample_count, rx_sample_count_next;
  logic [15:0] rx_sample_vec;
  always_ff @(posedge clk or posedge rst) begin
    if (rst) begin
      rx_sample_count <= 4'd0;
      rx_sample_vec   <= 16'hffff;
    end else if (rx_tick) begin
      rx_sample_count <= rx_sample_count_next;
      rx_sample_vec   <= {rx_sync_1, rx_sample_vec[15:1]};
    end
  end

  // RX State Machine
  typedef enum logic [2:0] {
    RX_IDLE,
    RX_DATA,
    RX_STOP
  } rx_state_t;
  rx_state_t rx_state, rx_state_next;
  logic [2:0] rx_bit_index, rx_bit_index_next;
  logic [7:0] rx_shift_reg, rx_shift_reg_next;

  always_comb begin : rx_fsm_comb
    rx_state_next        = rx_state;
    rx_bit_index_next    = rx_bit_index;
    rx_shift_reg_next    = rx_shift_reg;
    data_stream_out      = 0;
    data_stream_out_stb  = 1'b0;
    rx_sample_count_next = rx_sample_count + 1;

    case (rx_state)
      RX_IDLE: begin
        if (rx_sample_vec == 16'b0000_0000_0000_0001) begin  // Start bit detected
          rx_state_next        = RX_DATA;
          rx_bit_index_next    = 3'd0;
          rx_sample_count_next = 4'd0;
        end
      end

      RX_DATA: begin
        if (rx_sample_count == 4'd8) begin  // Sample in the middle of the bit
          rx_shift_reg_next[rx_bit_index] = rx_sync_1;
          if (rx_bit_index == 3'd7) begin
            rx_state_next = RX_STOP;
          end else begin
            rx_bit_index_next = rx_bit_index + 1;
          end
        end
      end

      RX_STOP: begin
        if (rx_sample_count == 4'd8) begin  // Sample stop bit
          if (rx_sync_1) begin  // Stop bit should be high
            data_stream_out     = rx_shift_reg;
            data_stream_out_stb = rx_tick;
            rx_state_next       = RX_IDLE;
            rx_bit_index_next   = 0;
          end else begin
            rx_state_next = RX_IDLE;  // Framing error, go back to idle
          end
        end
      end
      default: rx_state_next = RX_IDLE;
    endcase
  end

  always_ff @(posedge clk or posedge rst) begin
    if (rst) begin
      rx_state     <= RX_IDLE;
      rx_bit_index <= 3'b0;
      rx_shift_reg <= 8'b0;
    end else if (rx_tick) begin
      rx_state     <= rx_state_next;
      rx_bit_index <= rx_bit_index_next;
      rx_shift_reg <= rx_shift_reg_next;
    end
  end

  localparam integer TX_CLKS_PER_BIT = P_CLOCK_FREQUENCY / P_BAUD_RATE;
  logic [$clog2(TX_CLKS_PER_BIT)-1:0] tx_tick_counter;
  logic tx_tick;

  always_ff @(posedge clk or posedge rst) begin
    if (rst) begin
      tx_tick_counter <= '0;
      tx_tick         <= 1'b0;
    end else if (tx_tick_counter == TX_CLKS_PER_BIT - 1) begin
      tx_tick_counter <= '0;
      tx_tick         <= 1'b1;
    end else begin
      tx_tick_counter <= tx_tick_counter + 1;
      tx_tick         <= 1'b0;
    end
  end

  // TX State Machine
  typedef enum logic [2:0] {
    TX_IDLE,
    TX_START,
    TX_DATA,
    TX_STOP
  } tx_state_t;
  tx_state_t tx_state, tx_state_next;
  logic [2:0] tx_bit_index, tx_bit_index_next;
  logic tx_next, data_stream_in_ack_next;
  logic [7:0] tx_data, tx_data_next;

  always_comb begin : tx_fsm_comb
    tx_state_next           = tx_state;
    tx_bit_index_next       = 3'd0;
    tx_next                 = 1'b1;  // Line idle high
    tx_data_next            = tx_data;
    data_stream_in_ack_next = 1'b0;

    case (tx_state)
      TX_IDLE: begin
        if (data_stream_in_stb) begin
          tx_state_next = TX_START;
          tx_data_next = data_stream_in;
        end else begin
          tx_data_next = 8'b0;
        end
      end

      TX_START: begin
        tx_next = 1'b0; // Start bit
        tx_state_next    = TX_DATA;
      end

      TX_DATA: begin
        tx_next = tx_data[tx_bit_index];
        if (tx_bit_index == 3'd7) begin
          tx_state_next = TX_STOP;
        end else begin
          tx_bit_index_next = tx_bit_index + 1;
        end
      end

      TX_STOP: begin
        tx_next                 = 1'b1;  // Stop bit
        tx_state_next           = TX_IDLE;
        data_stream_in_ack_next = 1'b1;
        tx_data_next            = 8'b0;
      end

      default: tx_state_next = TX_IDLE;
    endcase
  end

  always_ff @(posedge clk or posedge rst) begin
    if (rst) begin
      tx_state           <= TX_IDLE;
      tx                 <= 1'b1;
      tx_bit_index       <= 3'd0;
      tx_data            <= 8'b0;
      data_stream_in_ack <= 1'b1;
    end else if (tx_tick) begin
      tx_state           <= tx_state_next;
      tx                 <= tx_next;
      tx_bit_index       <= tx_bit_index_next;
      tx_data            <= tx_data_next;
      data_stream_in_ack <= data_stream_in_ack_next;
    end else begin
      data_stream_in_ack <= 1'b0;
    end
  end

endmodule