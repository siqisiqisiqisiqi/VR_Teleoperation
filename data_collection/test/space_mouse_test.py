# import pyspacemouse
# import time

# success = pyspacemouse.open(
#     dof_callback=pyspacemouse.print_state, button_callback=pyspacemouse.print_buttons)
# if success:
#     while 1:
#         state = pyspacemouse.read()
#         print(f"state value is {state}.")
#         time.sleep(0.1)


# import pyspacemouse
# import time

# success = pyspacemouse.open()
# if success:
#     while 1:
#         state = pyspacemouse.read()
#         print(state.x, state.y, state.z)
#         time.sleep(0.01)


import pyspacemouse
import time


def button_0(state, buttons, pressed_buttons):
    print("Button:", pressed_buttons)


def button_1(state, buttons, pressed_buttons):
    print("Buttons:", pressed_buttons)


def someButton(state, buttons):
    print("Buttons:", buttons)


def callback():
    button_arr = [pyspacemouse.ButtonCallback(0, button_0),
                  pyspacemouse.ButtonCallback(14, button_1), ]

    success = pyspacemouse.open(dof_callback=pyspacemouse.print_state, button_callback=someButton,
                                button_callback_arr=button_arr)
    if success:
        while True:
            pyspacemouse.read()
            time.sleep(0.01)


if __name__ == '__main__':
    callback()
