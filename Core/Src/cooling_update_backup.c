/* Corre no main loop: pega na temperatura mais alta e aplica o PWM */
void Cooling_Update(void) {
	if (!inv_temps_updated && !ams_temps_updated)
		return;
	
	if (inv_temps_updated) {
		inv_temps_updated = 0;

	static uint8_t system_in_error = 1;

	float max_temp = MaxOf4(inv1_temp_inverter_c, inv1_temp_motor_c, inv2_temp_inverter_c, inv2_temp_motor_c);

	/* Histerese: liga ao atingir COOLING_TEMP_MIN_C, só desliga abaixo de
	 (COOLING_TEMP_MIN_C - COOLING_HYSTERESIS_C) - evita oscilar à volta do limiar */
	if (!cooling_active) {
		if (max_temp >= COOLING_TEMP_MIN_C) {
			cooling_active = 1;
		}
	} else {
		if (max_temp < (COOLING_TEMP_MIN_C - COOLING_HYSTERESIS_C)) {
			cooling_active = 0;
		}
	}

	/* Histerese AMS: liga a 50°C, desliga abaixo de (50 - COOLING_HYSTERESIS_C) */
	if (ams_temps_updated) {
		ams_temps_updated = 0;
		if (!ams_cooling_active) {
			if (ams_overall_max_temp_c >= 50.0f) {
				ams_cooling_active = 1;
			}
		} else {
			if (ams_overall_max_temp_c < (50.0f - COOLING_HYSTERESIS_C)) {
				ams_cooling_active = 0;
			}
		}
	}

	/* Ventoinhas AMS (Bateria) */
	if (ams_cooling_active) {
		HAL_GPIO_WritePin(GPIOB, AMS_Pin, GPIO_PIN_RESET); // ON (Ativo Baixo)
	} else {
		HAL_GPIO_WritePin(GPIOB, AMS_Pin, GPIO_PIN_SET);   // OFF
	}

	if (system_in_error) {
		system_in_error = 0;
	}

	uint8_t novo_pump = 0;
	uint8_t novo_fan = 0;

	if (cooling_active)
		Cooling_LookupPWM(max_temp, &novo_pump, &novo_fan);

	// TEMPORÁRIO: Forçar bomba a 40% para sangrar o circuito de água
	// Descomentar ou alterar aqui conforme necessário.
	//novo_pump = 30;

	/* Zona morta: com ruído no sensor a temperatura oscila alguns décimos e
	   o PWM ficava a tremer. Só reaplica se mudar o suficiente, ou se for
	   um extremo (0% / 100%) que tem de ser exato. */
	int delta_pump = (int) novo_pump - (int) pump_pwm_now;
	int delta_fan = (int) novo_fan - (int) fan_pwm_now;
	if (delta_pump < 0)
		delta_pump = -delta_pump;
	if (delta_fan < 0)
		delta_fan = -delta_fan;

	if (delta_pump >= COOLING_PWM_DEADBAND || novo_pump == 0 || novo_pump == 100) {
		pump_pwm_now = novo_pump;
		WaterPump_SetPWM(pump_pwm_now);
	}
	if (delta_fan >= COOLING_PWM_DEADBAND || novo_fan == 0 || novo_fan == 100) {
		fan_pwm_now = novo_fan;
		Radiator_SetPWM(fan_pwm_now);
	}
}

/* Helper genérico de TX (evita repetir o código dos mailboxes) */
static void CAN_Send(uint16_t id, uint8_t *data, uint8_t dlc) {
	TxHeader.StdId = id;
	TxHeader.DLC = dlc;
	if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) > 0 && HAL_CAN_AddTxMessage(&hcan, &TxHeader, data, &TxMailbox) == HAL_OK) {
