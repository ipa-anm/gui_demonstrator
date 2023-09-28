from nicegui import app, ui
import webview

result = None

def open_file_dialog(window):
    global result
    file_types = ('Xml_files (*.xml)', 'All files (*.*)')

    result = window.create_file_dialog(webview.OPEN_DIALOG, allow_multiple=False, file_types=file_types)
    window.destroy()

def start_webview():
    global result
    window = webview.create_window('Pick File')
    webview.start(open_file_dialog, window)

    return result[0]


