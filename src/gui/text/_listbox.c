/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Deviation is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Deviation.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "lcd_page_props.h"

void _DrawListbox(struct guiObject *obj, u8 redraw_all)
{
    struct guiListbox *listbox = (struct guiListbox *)obj;
    u16 obj_x, cx, cy;

    if (redraw_all)
        GUI_DrawBackground(obj->box.x, obj->box.y, obj->box.width, obj->box.height);

    LCD_GetCharDimensions(LCD_SELECT_CHAR, &cx, &cy);

    for(s32 i = 0; i < listbox->entries_per_page; i++) {
        const char *str = listbox->string_cb(i + listbox->cur_pos, listbox->cb_data);
        obj_x = obj->box.x;
        if (i + listbox->cur_pos == listbox->selected) {
            LCD_PrintCharXY(obj_x, obj->box.y + listbox->text_height * i, LCD_SELECT_CHAR);
            obj_x += cx;
        }
        else {
            GUI_DrawBackground(obj_x, obj->box.y + listbox->text_height * i, obj->box.width, listbox->text_height);
        }
        LCD_PrintStringXY(obj_x, obj->box.y + listbox->text_height * i, str);
    }
}
