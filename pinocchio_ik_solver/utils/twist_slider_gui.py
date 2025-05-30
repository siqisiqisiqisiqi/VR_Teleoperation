from PyQt5.QtWidgets import QWidget, QSlider, QLabel, QVBoxLayout
from PyQt5.QtCore import Qt


class TwistSliderWidget(QWidget):
    def __init__(self, max_linear=0.2, max_angular=0.5):
        super().__init__()
        self.setWindowTitle("CLIK Twist Control (x, y, z, roll, pitch, yaw)")

        self.layout = QVBoxLayout()
        self.labels = []
        self.sliders = []

        self.names = ["x", "y", "z", "roll", "pitch", "yaw"]
        self.max_vals = [max_linear] * 3 + [max_angular] * 3

        for i, name in enumerate(self.names):
            label = QLabel(f"{name}: 0.0")
            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(-1000)
            slider.setMaximum(1000)
            slider.setValue(0)
            slider.setSingleStep(1)

            slider.valueChanged.connect(
                lambda val, l=label, i=i: self.update_label(i, val, l))
            self.labels.append(label)
            self.sliders.append(slider)

            self.layout.addWidget(label)
            self.layout.addWidget(slider)

        self.setLayout(self.layout)

    def update_label(self, idx, val, label):
        value = (val / 1000.0) * self.max_vals[idx]
        label.setText(f"{self.names[idx]}: {value:.3f}")

    def get_twist(self):
        twist = [0.0] * 6
        for i, slider in enumerate(self.sliders):
            val = slider.value()
            twist[i] = (val / 1000.0) * self.max_vals[i]
        return twist


class PoseSliderWidget(QWidget):
    def __init__(self, initial_values, max_vals, names):
        super().__init__()
        self.setWindowTitle("Pose Control Sliders (x, y, z, roll, pitch, yaw)")

        self.layout = QVBoxLayout()
        self.labels = []
        self.sliders = []
        self.values = initial_values
        self.max_vals = max_vals
        self.names = names

        for i, name in enumerate(self.names):
            label = QLabel(f"{name}: {self.values[i]:.2f}")
            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(-1000)
            slider.setMaximum(1000)
            slider.setValue(int((1000 * self.values[i]) / max_vals[i]))
            slider.valueChanged.connect(
                lambda val, i=i, l=label: self.update_label(i, val, l))

            self.labels.append(label)
            self.sliders.append(slider)

            self.layout.addWidget(label)
            self.layout.addWidget(slider)

        self.setLayout(self.layout)

    def update_label(self, idx, val, label):
        mapped_val = (val / 1000.0) * self.max_vals[idx]
        self.values[idx] = mapped_val
        label.setText(f"{self.names[idx]}: {mapped_val:.2f}")

    def get_pose_values(self):
        return self.values
