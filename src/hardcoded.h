void hardcoded_drink()
{
  shaker_servo.write(180);
  if (digitalRead(0) == LOW)
  {
    // Recipe recipe;
    // recipe.SetPumps(std::vector<float>{10, 5, 3, 5});
    // recipe.SetShakeTime(3);
    // Bartender bartender(shaker_servo);
    // bartender.Start(recipe);
    digitalWrite(pump_id_to_gpio[0], HIGH);
    digitalWrite(pump_id_to_gpio[1], HIGH);
    digitalWrite(pump_id_to_gpio[2], HIGH);
    digitalWrite(pump_id_to_gpio[3], HIGH);
    for (int i = 0; i < DRINK_TIME; ++i)
    {
      if (i > DRINK_TIME/2)
      {
        digitalWrite(pump_id_to_gpio[2], LOW);
      }
      if (i > DRINK_TIME/1.2)
      {
        digitalWrite(pump_id_to_gpio[1], LOW);
        digitalWrite(pump_id_to_gpio[3], LOW);
      }
      delay(1000);
    }
    digitalWrite(pump_id_to_gpio[0], LOW);
    shaker_servo.write(300);
    for (int i = 0; i < AMOUNT_OF_BELTS; ++i)
    {
      digitalWrite(16, HIGH);
      delay(20);
      digitalWrite(16, LOW);
      delay(20);
    }
    shaker_servo.write(180);
    }
};