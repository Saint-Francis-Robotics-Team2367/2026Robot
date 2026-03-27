#pragma once

#include <frc/AddressableLED.h>
#include <frc/util/Color.h>
#include <frc/LEDPattern.h>
#include <vector>

class Led {
public:
    // Constructor
    Led(int pwmPort, int ledCount) : m_led(pwmPort), m_ledCount(ledCount), m_ledBuffer(ledCount), m_savedBuffer(ledCount) {
        m_led.SetLength(m_ledCount);
        for (auto& led : m_ledBuffer) {
            led.SetRGB(0, 0, 0); // All LEDs start turned off
        }
        m_led.SetData(m_ledBuffer);
        m_led.Start();
    }

    // Toggle LED strip on/off
    void Toggle() {
        if (m_isOn) {
            m_savedBuffer = m_ledBuffer;
            TurnOff();
        } else {
            m_ledBuffer = m_savedBuffer;
            m_led.SetData(m_ledBuffer);
        }
        m_isOn = !m_isOn;
    }

    // Set all LEDs to a solid color
    void SetSolidColor(frc::Color color){
        if (!m_isOn) return; // Do nothing if LEDs are off
        for (auto& led : m_ledBuffer) {
            led.SetRGB(color.red * 255, color.green * 255, color.blue * 255);
        }
        m_led.SetData(m_ledBuffer);
    }

    // Set an individual LED's color
    void SetLED(int index, frc::Color color){
        if (!m_isOn || index < 0 || index >= m_ledCount) {
            return; // Avoid out-of-range errors or disabled state
        }

        m_ledBuffer[index].SetRGB(color.red * 255, color.green * 255, color.blue * 255);
        m_led.SetData(m_ledBuffer);
    }

    // Apply a rainbow pattern
    void SetRainbowPattern(){
        if (!m_isOn) return; // Do nothing if LEDs are off

        for (int i = 0; i < m_ledCount; i++) {
            // Calculate the hue for each LED
            int hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledCount)) % 180;

            // Convert HSV to RGB and set the LED
            m_ledBuffer[i].SetHSV(hue, 255, 128);
        }
        // Increment hue for animation effect
        m_rainbowFirstPixelHue += 3;
        m_rainbowFirstPixelHue %= 180;

        m_led.SetData(m_ledBuffer);
    }

    // Update the LED buffer and apply changes
    void Update(){
        m_led.SetData(m_ledBuffer);
    }

    // Set Color function that uses new LEDPattern API
    void Set_Color(frc::Color color){
        frc::LEDPattern pattern = frc::LEDPattern::Solid(color);
        pattern.ApplyTo(m_ledBuffer);
        m_led.SetData(m_ledBuffer);
    }

    //*****************
    //IMPORTANT This should tell status of something on robot like height of elevator or something. idk how to use cause where do i get that info from
    void Progress_Mask();

    // Sets a continuous gradient of colors
    void Continuous_Gradient(frc::Color c1, frc::Color c2){
        std::array<frc::Color, 2> colors{c1, c2};
        frc::LEDPattern gradient = frc::LEDPattern::Gradient(frc::LEDPattern::GradientType::kContinuous, colors);
        // Apply the LED pattern to the data buffer
        gradient.ApplyTo(m_ledBuffer);
        // Write the data to the LED strip
        m_led.SetData(m_ledBuffer);
    }

private:
    frc::AddressableLED m_led;                                // LED controller
    std::vector<frc::AddressableLED::LEDData> m_ledBuffer;     // LED data buffer
    std::vector<frc::AddressableLED::LEDData> m_savedBuffer;   // Save state buffer for Toggle
    bool m_isOn = true;                                        // Tracks LED toggle state
    int m_ledCount;                                           // Number of LEDs
    int m_rainbowFirstPixelHue = 0;                           // For rainbow effect

    // Helper method to turn off all LEDs
    void TurnOff(){
        for (auto& led : m_ledBuffer) {
            led.SetRGB(0, 0, 0); // Set all LEDs to black (off)
        }
        m_led.SetData(m_ledBuffer);
    }
};