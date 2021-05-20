due_build:
	arduino-cli compile --fqbn arduino:sam:arduino_due_x_dbg -e src/due
due_flash:
	arduino-cli upload -p $(port) --fqbn arduino:sam:arduino_due_x_dbg -v src/due
due_reset:
	stty -F $(port) 1200
	sleep 1

due_update: due_build due_reset due_flash
