import tkinter as tk
from tkterminal import Terminal
from rich.console import Console
from rich.table import Table
import subprocess
import threading

class BashApp(tk.Tk):
    def __init__(self):
        super().__init__()

        self.title("R2-G2 Terminal")

        # Crear un terminal embebido usando tkterminal
        self.terminal = Terminal(self, width=80, height=24)
        self.terminal.pack(padx=10, pady=10, fill="both", expand=True)

        # Crear un botón para ejecutar comandos
        self.run_button = tk.Button(self, text="Ejecutar comando", command=self.run_command)
        self.run_button.pack(pady=5)

        # Crear un objeto de Rich Console
        self.console = Console()

        # Insertar texto de bienvenida en el terminal embebido
        self.terminal.insert(tk.END, "\nBienvenido al terminal R2-G2!\n")

    def run_command(self):
        """Ejecutar un comando y mostrar la salida en el terminal."""
        # Ejecutar el comando en un hilo separado para no bloquear la GUI
        threading.Thread(target=self.execute_command).start()

    def execute_command(self):
        """Ejecutar un comando de ejemplo y mostrar la salida."""
        # Crear una tabla de Rich como ejemplo
        table = Table(title="Estado del Robot")
        table.add_column("Componente", justify="right", style="cyan", no_wrap=True)
        table.add_column("Estado", justify="right", style="magenta")
        table.add_row("Cámara", "Encendida")
        table.add_row("Detección de Voz", "Apagada")
        table.add_row("Batería", "75%")

        # Capturar la salida en texto plano
        text = self.console.export_text(table)

        # Insertar el texto capturado en el terminal
        self.terminal.insert(tk.END, text + "\n")

        # También puedes ejecutar un comando externo
        process = subprocess.Popen(
            "echo Hola desde el terminal!",
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )

        for line in process.stdout:
            self.terminal.insert(tk.END, line)

        for line in process.stderr:
            self.terminal.insert(tk.END, line)

# Ejecutar la aplicación
if __name__ == "__main__":
    app = BashApp()
    app.mainloop()
