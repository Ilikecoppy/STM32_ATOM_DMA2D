// 4.16.0 0xa873a580
// Generated by imageconverter. Please, do not edit!

#include <BitmapDatabase.hpp>
#include <touchgfx/Bitmap.hpp>

extern const unsigned char image_blue_buttons_round_edge_icon_button[]; // BITMAP_BLUE_BUTTONS_ROUND_EDGE_ICON_BUTTON_ID = 0, Size: 60x60 pixels
extern const unsigned char image_blue_buttons_round_edge_icon_button_pressed[]; // BITMAP_BLUE_BUTTONS_ROUND_EDGE_ICON_BUTTON_PRESSED_ID = 1, Size: 60x60 pixels
extern const unsigned char image_logo[]; // BITMAP_LOGO_ID = 2, Size: 128x128 pixels
extern const unsigned char image_logogo[]; // BITMAP_LOGOGO_ID = 3, Size: 320x240 pixels

const touchgfx::Bitmap::BitmapData bitmap_database[] =
{
    { image_blue_buttons_round_edge_icon_button, 0, 60, 60, 7, 6, 46, (uint8_t)(touchgfx::Bitmap::ARGB8888) >> 3, 46, (uint8_t)(touchgfx::Bitmap::ARGB8888) & 0x7 },
    { image_blue_buttons_round_edge_icon_button_pressed, 0, 60, 60, 7, 6, 46, (uint8_t)(touchgfx::Bitmap::ARGB8888) >> 3, 46, (uint8_t)(touchgfx::Bitmap::ARGB8888) & 0x7 },
    { image_logo, 0, 128, 128, 52, 86, 13, (uint8_t)(touchgfx::Bitmap::ARGB8888) >> 3, 12, (uint8_t)(touchgfx::Bitmap::ARGB8888) & 0x7 },
    { image_logogo, 0, 320, 240, 0, 0, 320, (uint8_t)(touchgfx::Bitmap::RGB565) >> 3, 240, (uint8_t)(touchgfx::Bitmap::RGB565) & 0x7 }
};

namespace BitmapDatabase
{
const touchgfx::Bitmap::BitmapData* getInstance()
{
    return bitmap_database;
}

uint16_t getInstanceSize()
{
    return (uint16_t)(sizeof(bitmap_database) / sizeof(touchgfx::Bitmap::BitmapData));
}
}
