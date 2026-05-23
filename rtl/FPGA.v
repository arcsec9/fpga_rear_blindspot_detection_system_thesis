module fpga_ard(clk, rst_n, data_in, strobe, data_ready, sensor_select, ultrasonic_left_out, ultrasonic_right_out, ultrasonic_rear_out, hb100x_out);
	
	//FPGA 
	input wire clk;
	input wire rst_n;
	
	//FPGA TO ESP32_1
	input wire [9:0] data_in;
	input wire strobe;
	output reg data_ready;
	output reg [1:0] sensor_select;
	
	//FPGA TO ESP32_2
	output reg ultrasonic_left_out; // 1-BIT OUTPUT OF US LEFT
	output reg ultrasonic_right_out; // 1-BIT OUTPUT OF US RIGHT
	output reg [2:0] ultrasonic_rear_out; // 2-BIT OUTPUT OF US REAR
	output reg [2:0] hb100x_out; // 2-BIT OUTPUT OF HB100X
	
	//STATE DEFINITIONS
	localparam STARTUP_DELAY = 3'b000;
	localparam IDLE = 3'b001;
	localparam REQUEST_DATA = 3'b010;
	localparam WAIT_STROBE = 3'b011;
	localparam PROCESS_DATA = 3'b100;
	localparam UPDATE_SELECT = 3'b101;
	localparam RECOVERY = 3'b110;
	
	//sensor_select CYCLE FPGA TO ESP32_1
	localparam SELECT_HB100X = 2'b00;
	localparam SELECT_REAR = 2'b01;
	localparam SELECT_LEFT = 2'b10;
	localparam SELECT_RIGHT = 2'b11;
	
	//THRESHOLD OF HB100X FREQUENCY TO 10-BIT VALUE
	localparam HB100X_THRESH_2KMH = 10'd38;
	localparam HB100X_THRESH_5KMH = 10'd96;
	localparam HB100X_THRESH_10KMH = 10'd193;
	localparam HB100X_THRESH_15KMH = 10'd289;
	localparam HB100X_THRESH_20KMH = 10'd386;
	localparam HB100X_THRESH_25KMH = 10'd482;
	localparam HB100X_THRESH_30KMH = 10'd579;
	
	//THRESHOLD FOR ULTRASONIC SENSORS (LEFT, REAR, AND RIGHT)
	localparam ULTRASONIC_THRESH_10CM = 10'd21;
	localparam ULTRASONIC_THRESH_50CM = 10'd102;
	localparam ULTRASONIC_THRESH_75CM = 10'd153;  // Added intermediate threshold
	localparam ULTRASONIC_THRESH_100CM = 10'd205;
	localparam ULTRASONIC_THRESH_125CM = 10'd256;  // Added intermediate threshold
	localparam ULTRASONIC_THRESH_150CM = 10'd307;
	localparam ULTRASONIC_THRESH_200CM = 10'd410;
	localparam ULTRASONIC_THRESH_250CM = 10'd512;
	localparam ULTRASONIC_THRESH_300CM = 10'd614;
	localparam ULTRASONIC_THRESH_350CM = 10'd716;  // Added intermediate threshold
	localparam ULTRASONIC_THRESH_400CM = 10'd819;
	localparam ULTRASONIC_THRESH_450CM = 10'd921;  // Added intermediate threshold
	localparam ULTRASONIC_THRESH_500CM = 10'd1023;
	
	//STATE REGISTER
	reg [2:0] state, next_state;
	
	//DATA REGISTERS
	reg [9:0] hb100x_data;
	reg [9:0] ultrasonic_rear_data;
	reg [9:0] ultrasonic_left_data;
	reg [9:0] ultrasonic_right_data;

	//STARTUP DELAY COUNTER BASED ON 55MHz
	reg [23:0] startup_counter;
	localparam STARTUP_COUNT = 24'd55000; // ~1ms at 55MHz (reduced from 10ms)
	
	//TIMEOUT COUNTER
	reg [19:0] timeout_counter;
	localparam TIMEOUT_COUNT = 20'd5500; // ~0.1ms at 55MHz (reduced from 0.5ms)
	
	//RECOVERY COUNTER
	reg [23:0] recovery_counter;
	localparam RECOVERY_COUNT = 24'd55000; // ~1ms at 55MHz (reduced from 5ms)
	
	// Watchdog counter
	reg [27:0] watchdog_counter;
	localparam WATCHDOG_COUNT = 28'd550000; // ~10ms at 55MHz (reduced from 100ms)
	
	// Consecutive timeout counter
	reg [3:0] consecutive_timeouts;
	localparam MAX_CONSECUTIVE_TIMEOUTS = 4'd2; // Reduced from 3 for faster recovery
	
	// Strobe detection with minimal debouncing
	reg strobe_prev;
	wire strobe_rising = (strobe_prev == 1'b0 && strobe == 1'b1);
	
	// Request delay counter
	reg [19:0] request_delay_counter;
	localparam REQUEST_DELAY_COUNT = 20'd550; // ~0.01ms at 55MHz (reduced from 0.1ms)
	
	// Data valid flags
	reg hb100x_data_valid;
	reg ultrasonic_rear_data_valid;
	reg ultrasonic_left_data_valid;
	reg ultrasonic_right_data_valid;
	
	// State machine
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			state <= STARTUP_DELAY;
			startup_counter <= 24'd0;
			timeout_counter <= 20'd0;
			recovery_counter <= 24'd0;
			watchdog_counter <= 28'd0;
			request_delay_counter <= 20'd0;
			consecutive_timeouts <= 4'd0;
			strobe_prev <= 1'b0;
			data_ready <= 1'b0;
			sensor_select <= SELECT_HB100X;
			
			hb100x_data <= 10'd0;
			ultrasonic_rear_data <= 10'd0;
			ultrasonic_left_data <= 10'd0;
			ultrasonic_right_data <= 10'd0;
			
			hb100x_data_valid <= 1'b0;
			ultrasonic_rear_data_valid <= 1'b0;
			ultrasonic_left_data_valid <= 1'b0;
			ultrasonic_right_data_valid <= 1'b0;
			
			hb100x_out <= 3'b000;
			ultrasonic_rear_out <= 3'b000;
			ultrasonic_left_out <= 1'b0;
			ultrasonic_right_out <= 1'b0;
		end else begin
			// Single flip-flop for strobe detection
			strobe_prev <= strobe;
			
			// Watchdog counter always increments
			watchdog_counter <= watchdog_counter + 1'b1;
			
			// If watchdog expires, force a reset to RECOVERY state
			if (watchdog_counter >= WATCHDOG_COUNT) begin
				watchdog_counter <= 28'd0;
				state <= RECOVERY;
				data_ready <= 1'b0;
				recovery_counter <= 24'd0;
			end else begin
				state <= next_state;
				
				case (state)
					STARTUP_DELAY: begin
						startup_counter <= startup_counter + 1'b1;
						data_ready <= 1'b0;
					end

					IDLE: begin
						data_ready <= 1'b0;
						timeout_counter <= 20'd0;
						request_delay_counter <= request_delay_counter + 1'b1;
						watchdog_counter <= 28'd0;
					end

					REQUEST_DATA: begin
						data_ready <= 1'b1;
						timeout_counter <= 20'd0;
						request_delay_counter <= 20'd0;
					end
					
					WAIT_STROBE: begin
						timeout_counter <= timeout_counter + 1'b1;
						
						if (strobe_rising) begin
							case (sensor_select)
								SELECT_HB100X: begin
									hb100x_data <= data_in;
									hb100x_data_valid <= 1'b1;
								end
								SELECT_REAR: begin
									ultrasonic_rear_data <= data_in;
									ultrasonic_rear_data_valid <= 1'b1;
								end
								SELECT_LEFT: begin
									ultrasonic_left_data <= data_in;
									ultrasonic_left_data_valid <= 1'b1;
								end
								SELECT_RIGHT: begin
									ultrasonic_right_data <= data_in;
									ultrasonic_right_data_valid <= 1'b1;
								end
							endcase
							consecutive_timeouts <= 4'd0;
						end
					end
							
					PROCESS_DATA: begin
						data_ready <= 1'b0;
						
						if (timeout_counter >= TIMEOUT_COUNT) begin
							consecutive_timeouts <= consecutive_timeouts + 1'b1;
						end
						
						if (hb100x_data_valid) begin
							if (hb100x_data <= HB100X_THRESH_2KMH)
								hb100x_out <= 3'b000;
							else if (hb100x_data <= HB100X_THRESH_5KMH)
								hb100x_out <= 3'b001;
							else if (hb100x_data <= HB100X_THRESH_10KMH)
								hb100x_out <= 3'b010;
							else if (hb100x_data <= HB100X_THRESH_15KMH)
								hb100x_out <= 3'b011;
							else if (hb100x_data <= HB100X_THRESH_20KMH)
								hb100x_out <= 3'b100;
							else if (hb100x_data <= HB100X_THRESH_25KMH)
								hb100x_out <= 3'b101;
							else if (hb100x_data <= HB100X_THRESH_30KMH)
								hb100x_out <= 3'b110;
							else
								hb100x_out <= 3'b111;
						end
						
						if (ultrasonic_rear_data_valid) begin
							// Speed-based distance thresholds
							if (hb100x_data <= HB100X_THRESH_10KMH) begin
								// Low speed thresholds (0-10 km/h)
								if (ultrasonic_rear_data <= ULTRASONIC_THRESH_50CM)
									ultrasonic_rear_out <= 3'b111;  // Critical warning
								else if (ultrasonic_rear_data <= ULTRASONIC_THRESH_75CM)
									ultrasonic_rear_out <= 3'b110;  // High warning
								else if (ultrasonic_rear_data <= ULTRASONIC_THRESH_100CM)
									ultrasonic_rear_out <= 3'b101;  // Medium warning
								else if (ultrasonic_rear_data <= ULTRASONIC_THRESH_150CM)
									ultrasonic_rear_out <= 3'b100;  // Low warning
								else if (ultrasonic_rear_data <= ULTRASONIC_THRESH_200CM)
									ultrasonic_rear_out <= 3'b011;  // Caution
								else if (ultrasonic_rear_data <= ULTRASONIC_THRESH_300CM)
									ultrasonic_rear_out <= 3'b010;  // Safe
								else if (ultrasonic_rear_data <= ULTRASONIC_THRESH_400CM)
									ultrasonic_rear_out <= 3'b001;  // Very safe
								else
									ultrasonic_rear_out <= 3'b000;  // Clear
							end else if (hb100x_data <= HB100X_THRESH_20KMH) begin
								// Medium speed thresholds (10-20 km/h)
								if (ultrasonic_rear_data <= ULTRASONIC_THRESH_75CM)
									ultrasonic_rear_out <= 3'b111;  // Critical warning
								else if (ultrasonic_rear_data <= ULTRASONIC_THRESH_100CM)
									ultrasonic_rear_out <= 3'b110;  // High warning
								else if (ultrasonic_rear_data <= ULTRASONIC_THRESH_150CM)
									ultrasonic_rear_out <= 3'b101;  // Medium warning
								else if (ultrasonic_rear_data <= ULTRASONIC_THRESH_200CM)
									ultrasonic_rear_out <= 3'b100;  // Low warning
								else if (ultrasonic_rear_data <= ULTRASONIC_THRESH_300CM)
									ultrasonic_rear_out <= 3'b011;  // Caution
								else if (ultrasonic_rear_data <= ULTRASONIC_THRESH_400CM)
									ultrasonic_rear_out <= 3'b010;  // Safe
								else if (ultrasonic_rear_data <= ULTRASONIC_THRESH_500CM)
									ultrasonic_rear_out <= 3'b001;  // Very safe
								else
									ultrasonic_rear_out <= 3'b000;  // Clear
							end else begin
								// High speed thresholds (>20 km/h)
								if (ultrasonic_rear_data <= ULTRASONIC_THRESH_100CM)
									ultrasonic_rear_out <= 3'b111;  // Critical warning
								else if (ultrasonic_rear_data <= ULTRASONIC_THRESH_150CM)
									ultrasonic_rear_out <= 3'b110;  // High warning
								else if (ultrasonic_rear_data <= ULTRASONIC_THRESH_200CM)
									ultrasonic_rear_out <= 3'b101;  // Medium warning
								else if (ultrasonic_rear_data <= ULTRASONIC_THRESH_300CM)
									ultrasonic_rear_out <= 3'b100;  // Low warning
								else if (ultrasonic_rear_data <= ULTRASONIC_THRESH_400CM)
									ultrasonic_rear_out <= 3'b011;  // Caution
								else if (ultrasonic_rear_data <= ULTRASONIC_THRESH_500CM)
									ultrasonic_rear_out <= 3'b010;  // Safe
								else
									ultrasonic_rear_out <= 3'b000;  // Clear
							end
						end
						
						if (ultrasonic_left_data_valid) begin
							ultrasonic_left_out <= (ultrasonic_left_data >= ULTRASONIC_THRESH_10CM && 
												  ultrasonic_left_data <= ULTRASONIC_THRESH_500CM);
						end
						
						if (ultrasonic_right_data_valid) begin
							ultrasonic_right_out <= (ultrasonic_right_data >= ULTRASONIC_THRESH_10CM && 
												   ultrasonic_right_data <= ULTRASONIC_THRESH_500CM);
						end
					end

					UPDATE_SELECT: begin
						if (consecutive_timeouts >= MAX_CONSECUTIVE_TIMEOUTS) begin
							state <= RECOVERY;
							recovery_counter <= 24'd0;
							consecutive_timeouts <= 4'd0;
						end else begin
							case (sensor_select)
								SELECT_HB100X: sensor_select <= SELECT_REAR;
								SELECT_REAR:   sensor_select <= SELECT_LEFT;
								SELECT_LEFT:   sensor_select <= SELECT_RIGHT;
								SELECT_RIGHT:  sensor_select <= SELECT_HB100X;
							endcase
						end
					end
	  
					RECOVERY: begin
						data_ready <= 1'b0;
						recovery_counter <= recovery_counter + 1'b1;
					end
				endcase
			end
		end
	end

	// State transitions
	always @(*) begin
		case (state)
			STARTUP_DELAY:   next_state = (startup_counter >= STARTUP_COUNT) ? IDLE : STARTUP_DELAY;
			IDLE:            next_state = (request_delay_counter >= REQUEST_DELAY_COUNT) ? REQUEST_DATA : IDLE;
			REQUEST_DATA:    next_state = WAIT_STROBE;
			WAIT_STROBE:     begin
				if (strobe_rising)
					next_state = PROCESS_DATA;
				else if (timeout_counter >= TIMEOUT_COUNT)
					next_state = PROCESS_DATA;
				else
					next_state = WAIT_STROBE;
			end
			PROCESS_DATA:    next_state = UPDATE_SELECT;
			UPDATE_SELECT:   next_state = IDLE;
			RECOVERY:        next_state = (recovery_counter >= RECOVERY_COUNT) ? IDLE : RECOVERY;
			default:         next_state = IDLE;
		endcase
	end

endmodule 