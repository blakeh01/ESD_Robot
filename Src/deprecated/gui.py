import random
import tkinter as tk
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg, NavigationToolbar2Tk)

# Main Control window, needs lots of work.
class TestGui:

    def __init__(self, main_instance):
        self.main_instance = main_instance
        self.main_window = tk.Tk()
        self.graph_window = WindowStateGraph([1, 1, 0, 2, 3])

        self.lbl_time = None
        self.lbl_dt = None
        self.lbl_mode = None
        self.btn_exit = None
        self.btn_shadow = None

        self.lbl_robot_status = None
        self.lbl_stepper_status = None

        self.init_main()

    def init_main(self):
        self.main_window.protocol("WM_DELETE_WINDOW", self.disable_event) # Disable the closing of the main window.
        self.main_window.geometry("300x400")

        self.lbl_time = tk.Label(master=self.main_window, text="Time Elapsed: 0")
        self.lbl_dt = tk.Label(master=self.main_window, text="Time Elapsed: 0")

        self.lbl_mode = tk.Label(master=self.main_window, text="Current Shadow: Robot")

        self.btn_exit = tk.Button(
            master=self.main_window,
            text="Shutdown",
            width=20,
            height=3,
            bg="red",
            fg="black",
            command=self.main_instance.shutdown
        )
        self.btn_shadow = tk.Button(
            master=self.main_window,
            text="Toggle Shadow Mode",
            width=20,
            height=3,
            bg="black",
            fg="white",
            command=self.main_instance.switch_mode
        )
        tk.Label(text="Robot status: ", master=self.main_window).pack()
        self.lbl_robot_status = tk.Label(text="ERROR", fg="black", bg="red")
        self.lbl_robot_status.pack()

        self.lbl_time.pack()
        self.lbl_dt.pack()
        self.lbl_mode.pack()
        self.btn_shadow.pack()
        self.btn_exit.pack()




    def update_gui(self, t=0, dt=0):
        self.main_window.update()
        self.graph_window.update_gui(t)
        self.lbl_time["text"] = f"Time Elapsed {round(t, 3)}"
        self.lbl_dt["text"] = f"  |  Process time: {round(dt, 3)}"

        if self.main_instance.shadow == 0:
            self.lbl_mode["text"] = f"Current Shadow: Robot"
        elif self.main_instance.shadow == 1:
            self.lbl_mode["text"] = f"Current Shadow: Simulation"
        else:
            self.lbl_mode["text"] = f"Current Shadow: None"

    def disable_event(self):
        pass


# Handles plotting joint state data to a new window.
class WindowStateGraph:

    def __init__(self, joint_state):
        self.joint_state = joint_state
        self.previous_state = joint_state

        self.cached_states = []

        self.figure = Figure(figsize=(5, 5), dpi=100)

        self.sp_shoulder = self.figure.add_subplot(111)
        self.sp_elbow = self.figure.add_subplot(111)

        self.window = tk.Tk()
        self.window.title("Joint States")
        self.btn_graph = None

        self.canvas = FigureCanvasTkAgg(self.figure, master=self.window)
        self.toolbar = NavigationToolbar2Tk(self.canvas, self.window)

        self.canvas.draw()
        self.toolbar.update()
        self.canvas.get_tk_widget().pack()

        self.enabled = False

    def update_gui(self, t=0):
        self.window.update()
        if not self.enabled:
            return

        self.graph()

    def graph(self):
        self.cached_states.append([random.randint(0, 10), random.randint(0, 10), random.randint(0, 10),
                                   random.randint(0, 10), random.randint(0, 10)])
        shoulder_states = []
        for state in self.cached_states:
            shoulder_states.append(state[2])

        self.sp_shoulder.plot(shoulder_states)
        self.canvas.draw_idle()
