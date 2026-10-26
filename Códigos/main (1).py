# main.py  -  Comunicacion digital inalambrica GFSK  (Raspberry Pi Pico 2W + NRF24L01)
#
# BOTONES (iguales en TODAS las pantallas)
#   GP13 corto  : siguiente opcion / +1
#   GP13 largo  : (menu) opcion anterior / (canal) +10
#   GP12 corto  : aceptar / entrar / enviar
#   GP12 largo  : ATRAS (mantener ~0.7 s). Vuelve sin reiniciar la Pico. En las
#                 pantallas se escribe "12L".
# LEDS: rojo = error / no se transmitio, amarillo = configurando / esperando,
#       verde = enviado / recibido.
from machine import Pin, SPI, I2C
import ssd1306, time, math, os
from nrf24 import NRF24, POT_DBM

# ================= Hardware =================
i2c = I2C(1, sda=Pin(14), scl=Pin(15), freq=400000)
oled = ssd1306.SSD1306_I2C(128, 64, i2c, addr=0x3C)
spi = SPI(0, baudrate=1000000, polarity=0, phase=0,
          sck=Pin(18), mosi=Pin(19), miso=Pin(16))
nrf = NRF24(spi, csn=Pin(17, Pin.OUT), ce=Pin(20, Pin.OUT))

b_var = Pin(13, Pin.IN, Pin.PULL_UP)     # variar entre opciones
b_sel = Pin(12, Pin.IN, Pin.PULL_UP)     # seleccionar / atras (largo)
led_r = Pin(11, Pin.OUT)
led_a = Pin(10, Pin.OUT)
led_v = Pin(9, Pin.OUT)

# ================= Parametros del sistema =================
ANT_DBI = 2                  # ganancia de la antena del modulo [dBi] (TX y RX). CONFIRMAR con el modulo
LN10 = 2.302585093
CANAL_MAX = 125              # canales 0..125 -> 2400..2525 MHz
LARGO_MS = 700               # tiempo para considerar "pulsacion larga"

# POTENCIA de TX (registro RF_PWR) y SENSIBILIDAD de RX son cosas distintas:
#  - la potencia se programa: -18, -12, -6 o 0 dBm
#  - la sensibilidad no se programa: depende de la velocidad (datasheet tabla 7)
#    y de la ganancia del LNA (LNA baja = 1.5 dB peor)
POT_TXT = ["-18 dBm", "-12 dBm", "-6 dBm", "0 dBm"]
SENS_BASE = {"250K": -94.0, "1M": -85.0, "2M": -82.0}   # 250K solo en el nRF24L01+
VEL_TXT = {"250K": "250kbps", "1M": "1Mbps", "2M": "2Mbps"}
VELS = ["1M", "2M"]          # se agrega "250K" al inicio si el modulo lo soporta
LNA_TXT = ["Baja", "Alta"]

ROLES = ["TX", "RX"]
PRUEBAS = ["Mensajes", "BER alfab.", "BER aleat."]      # modo de trabajo (corto)
PRUEBAS_LARGO = ["Mensajes", "BER alfabeto", "BER aleatorio"]
ENVIOS = ["Unico", "Loop 1 msg", "Loop lista", "Continuo"]
ENVIOS_INFO = ["1 envio con 12", "repite el msg", "recorre la lista", "para espectro"]
INTERVALOS = [100, 250, 500, 1000, 2000]               # ms entre envios del loop
IV_TXT = ["100 ms", "250 ms", "500 ms", "1 s", "2 s"]
PASADAS = [1, 5, 10, 20]                               # veces que se envia el archivo (BER)
MENSAJES = [b"ALEGRIA", b"MIGUELITO",
            b"ABCDEFGHIJKLMNOPQRSTUVWXYZ",
            b"abcdefghijklmnopqrstuvwxyz",
            b"ABCDEFGHIJKLMNOPQRSTUVWXYZ123456"]       # maximo 32 bytes

# ---- Prueba de BER (punto 4.3) ----
# Trama de 32 bytes:  [seq_hi, seq_lo, ~seq_hi, ~seq_lo] + 28 bytes del archivo
# El numero de secuencia (con su complemento) permite saber que trozo del archivo
# llego aunque falten paquetes, y detectar si el encabezado llego danado.
DATOS_BER = 28
FIN_HDR = b"\xff\xff\xff\xff"        # trama de fin: FIN_HDR + total(2) + pasadas(1) + checksum(2)
N_ARCHIVO = 28 * 64                  # 1792 bytes = 64 paquetes (si se generan los archivos)
PAUSA_BER_MS = 15                    # pausa entre paquetes (el RX necesita tiempo para el OLED)
ARCHIVO_CSV = "resultados_ber.csv"
POP = bytes([bin(i).count("1") for i in range(256)])   # bits en 1 de cada byte

# configuracion actual ("can" es el numero de canal directamente: 0..125)
cfg = {"rol": 0, "can": 40, "pot": 3, "vel": "1M", "lna": 1,
       "prueba": 0, "msg": 0, "env": 0, "iv": 3, "pas": 1}


# ================= Calculos teoricos (Friis) =================
def sens_dbm(vel=None, lna=None):
    vel = cfg["vel"] if vel is None else vel
    lna = cfg["lna"] if lna is None else lna
    return SENS_BASE[vel] + (0 if lna else 1.5)

def presupuesto_db(pot=None, vel=None, lna=None):
    """Perdida maxima admisible = Pt + Gt + Gr - Sensibilidad."""
    pot = cfg["pot"] if pot is None else pot
    return POT_DBM[pot] + 2 * ANT_DBI - sens_dbm(vel, lna)

def alcance_m(f_mhz, pot=None, vel=None, lna=None):
    """Alcance en espacio libre [m]:  d = 10^((P - 32.44 - 20log f)/20) km."""
    p = presupuesto_db(pot, vel, lna)
    return 1000.0 * 10 ** ((p - 32.44 - 20 * math.log(f_mhz) / LN10) / 20)

def fmt_dist(d):
    return "%d m" % round(d) if d < 1000 else "%.2f km" % (d / 1000)

def fmt_num(x):
    t = "%.1f" % x
    return t[:-2] if t.endswith(".0") else t


# ================= Botones y LEDs =================
def leds(r=0, a=0, v=0):
    led_r(r)
    led_a(a)
    led_v(v)

def leer_botones():
    """Devuelve 'var', 'var_largo', 'sel', 'sel_largo' o None (espera a que se suelte)."""
    for pin, nombre in ((b_var, "var"), (b_sel, "sel")):
        if pin.value() == 0:
            time.sleep_ms(30)                        # antirrebote
            t0 = time.ticks_ms()
            while pin.value() == 0 and time.ticks_diff(time.ticks_ms(), t0) < LARGO_MS:
                time.sleep_ms(10)
            largo = pin.value() == 0
            while pin.value() == 0:
                time.sleep_ms(10)
            return nombre + ("_largo" if largo else "")
    return None

def esperar_boton():
    ev = None
    while ev is None:
        ev = leer_botones()
        time.sleep_ms(10)
    return ev

def hay_boton():
    return b_var.value() == 0 or b_sel.value() == 0

def esperar_soltar():
    while hay_boton():
        time.sleep_ms(10)
    time.sleep_ms(30)


# ================= Pantalla =================
def mostrar(lineas):
    """Hasta 8 lineas de 16 caracteres (8 px por linea)."""
    oled.fill(0)
    for k, t in enumerate(lineas[:8]):
        oled.text(t[:16], 0, 8 * k)
    oled.show()

def cabecera(tag, ber=False):
    """3 primeras lineas de las pantallas de trabajo: canal/freq, velocidad, pot/sens."""
    c = cfg["can"]
    p = POT_DBM[cfg["pot"]]
    if ber:
        return ["%s Ch%d" % (tag, c),
                "%dMHz %s" % (2400 + c, VEL_TXT[cfg["vel"]]),
                "P:%d S:%sdBm" % (p, fmt_num(sens_dbm()))]
    return ["%s Ch%d %dMHz" % (tag, c, 2400 + c),
            "%s Ant %ddBi" % (VEL_TXT[cfg["vel"]], ANT_DBI),
            "P:%d S:%sdBm" % (p, fmt_num(sens_dbm()))]

def verificar_nrf():
    if not nrf.presente():
        leds(r=1)
        mostrar(["ERROR NRF24L01", "", "Revise cableado", "(SPI, CSN, CE)"])
        while True:
            time.sleep(1)

def inicio():
    global VELS
    leds(a=1)
    mostrar(["Comunicacion", "digital", "inalambrica", "", "GFSK - UMNG"])
    time.sleep_ms(2500)
    verificar_nrf()
    if nrf.soporta_250k():
        VELS = ["250K", "1M", "2M"]
    c = cfg["can"]
    mostrar(["CONFIGURACION",
             "Canal: %d" % c,
             "Freq: %d MHz" % (2400 + c),
             "Vel: " + VEL_TXT[cfg["vel"]],
             "Pot TX: " + POT_TXT[cfg["pot"]],
             "Sens RX: %s dBm" % fmt_num(sens_dbm()),
             "Antena: %d dBi" % ANT_DBI])
    time.sleep_ms(3000)
    mostrar(["BOTONES", "", "13: siguiente", "12: aceptar", "12L: ATRAS",
             "(L = mantener 1 s)"])
    time.sleep_ms(2500)

def aplicar_radio(ack, crc):
    """Escribe la configuracion en el NRF24L01, la lee de vuelta y la muestra."""
    leds(a=1)
    ok = nrf.aplicar(cfg["can"], cfg["pot"], cfg["vel"], cfg["lna"], ack, crc)
    c = nrf.leer_config()
    print("NRF24 leido:", c, "OK" if ok else "ERROR")
    mostrar(["REGISTROS NRF24",
             "RF_CH=%d %dMHz" % (c["canal"], 2400 + c["canal"]),
             "RF_SETUP=0x%02X" % nrf.read_reg(0x06),
             "%s Pot:%d" % (VEL_TXT[c["vel"]], c["pot"]),
             "LNA:%d ACK:%d CRC:%d" % (c["lna"], c["ack"], c["crc"]),
             "CONFIG OK" if ok else "ERROR CONFIG"])
    if not ok:
        leds(r=1)
    time.sleep_ms(1500)
    leds(a=1)
    return ok

def pantalla_alcance():
    c = cfg["can"]
    d_min = alcance_m(2525)
    d_max = alcance_m(2400)
    mostrar(["ALCANCE TEORICO",
             "Ant:%d dBi x2" % ANT_DBI,
             "Pot TX:" + POT_TXT[cfg["pot"]],
             "Sens RX:%s dBm" % fmt_num(sens_dbm()),
             "Presup: %.1f dB" % presupuesto_db(),
             "Ch%d: %s" % (c, fmt_dist(alcance_m(2400 + c))),
             "Min-Max:",
             "%s-%s" % (fmt_dist(d_min), fmt_dist(d_max))])
    esperar_boton()                                  # cualquier boton vuelve al menu


# ================= Menu =================
def claves_menu():
    """Lista de opciones; cambia segun el rol y el modo de trabajo."""
    k = ["rol", "can", "pot", "vel", "lna", "prueba"]
    if cfg["rol"] == 0:
        k += ["msg", "env", "iv"] if cfg["prueba"] == 0 else ["pas"]
    return k + ["alc", "ini"]

def etiqueta(k):
    if k == "rol":    return "Rol: " + ROLES[cfg["rol"]]
    if k == "can":    return "Can:%d %dMHz" % (cfg["can"], 2400 + cfg["can"])
    if k == "pot":    return "Pot TX:" + POT_TXT[cfg["pot"]]
    if k == "vel":    return "Vel: " + VEL_TXT[cfg["vel"]]
    if k == "lna":    return "LNA:%s S:%s" % (LNA_TXT[cfg["lna"]], fmt_num(sens_dbm()))
    if k == "prueba": return "Modo:" + PRUEBAS[cfg["prueba"]]
    if k == "msg":    return ("Msg %d/%d:" % (cfg["msg"] + 1, len(MENSAJES))) + MENSAJES[cfg["msg"]].decode()
    if k == "env":    return "Envio:" + ENVIOS[cfg["env"]]
    if k == "iv":     return "Interv:" + IV_TXT[cfg["iv"]]
    if k == "pas":    return "Pasadas: %d" % PASADAS[cfg["pas"]]
    if k == "alc":    return "Alcance teorico"
    return "INICIAR"

def dibujar_menu(claves, sel):
    n = len(claves)
    ini = max(0, min(sel - 2, n - 6))                # ventana de 6 filas
    oled.fill(0)
    oled.text("MENU", 0, 0)
    oled.text("%d/%d" % (sel + 1, n), 128 - 8 * len("%d/%d" % (sel + 1, n)), 0)
    for r in range(6):
        i = ini + r
        if i >= n:
            break
        y = 10 + 9 * r
        if i == sel:                                 # fila seleccionada en video inverso
            oled.fill_rect(0, y - 1, 128, 9, 1)
            oled.text(etiqueta(claves[i])[:16], 0, y, 0)
        else:
            oled.text(etiqueta(claves[i])[:16], 0, y)
    oled.show()

def elegir(titulo, opciones, idx):
    """opciones: lista de (linea1, linea2). 13: siguiente, 12: aceptar, 12L: atras
    (devuelve el indice original sin cambios)."""
    original = idx
    while True:
        l1, l2 = opciones[idx]
        mostrar([titulo + " %d/%d" % (idx + 1, len(opciones)), "", l1, l2, "",
                 "13: siguiente", "12: aceptar", "12L: atras"])
        ev = esperar_boton()
        if ev in ("var", "var_largo"):
            idx = (idx + 1) % len(opciones)
        elif ev == "sel":
            return idx
        elif ev == "sel_largo":
            return original

def elegir_canal(ch):
    """13: +1, 13 largo: +10, 12: aceptar, 12L: atras (sin cambios)."""
    original = ch
    while True:
        mostrar(["CANAL", "", "Canal: %d" % ch, "Freq: %d MHz" % (2400 + ch), "",
                 "13: +1  13L: +10", "12: aceptar", "12L: atras"])
        ev = esperar_boton()
        if ev == "var":
            ch = (ch + 1) % (CANAL_MAX + 1)
        elif ev == "var_largo":
            ch = (ch + 10) % (CANAL_MAX + 1)
        elif ev == "sel":
            return ch
        elif ev == "sel_largo":
            return original

def editar(k):
    if k == "rol":
        cfg["rol"] = elegir("ROL", [("TRANSMISOR", ""), ("RECEPTOR", "")], cfg["rol"])
    elif k == "can":
        cfg["can"] = elegir_canal(cfg["can"])
    elif k == "pot":          # potencia de TX, con el alcance teorico de cada nivel
        ops = [(POT_TXT[i], "Alcance " + fmt_dist(alcance_m(2400 + cfg["can"], pot=i)))
               for i in range(4)]
        cfg["pot"] = elegir("POTENCIA TX", ops, cfg["pot"])
    elif k == "vel":          # la sensibilidad depende de la velocidad
        ops = [(VEL_TXT[v], "Sens %s dBm" % fmt_num(sens_dbm(vel=v))) for v in VELS]
        cfg["vel"] = VELS[elegir("VELOCIDAD", ops, VELS.index(cfg["vel"]))]
    elif k == "lna":
        ops = [(LNA_TXT[n], "Sens %s dBm" % fmt_num(sens_dbm(lna=n))) for n in (0, 1)]
        cfg["lna"] = elegir("LNA (SENSIB.)", ops, cfg["lna"])
    elif k == "prueba":
        info = ["nombres/alfabeto", "alfabetos.txt", "aleatorio.txt"]
        cfg["prueba"] = elegir("MODO", list(zip(PRUEBAS_LARGO, info)), cfg["prueba"])
    elif k == "msg":
        cfg["msg"] = elegir("MENSAJE", [(m.decode()[:16], m.decode()[16:32]) for m in MENSAJES],
                            cfg["msg"])
    elif k == "env":
        cfg["env"] = elegir("MODO DE ENVIO", list(zip(ENVIOS, ENVIOS_INFO)), cfg["env"])
    elif k == "iv":
        cfg["iv"] = elegir("INTERVALO LOOP", [(t, "") for t in IV_TXT], cfg["iv"])
    elif k == "pas":
        cfg["pas"] = elegir("PASADAS ARCHIVO", [("%d" % p, "") for p in PASADAS], cfg["pas"])
    elif k == "alc":
        pantalla_alcance()

def menu():
    leds(a=1)
    sel = 0
    while True:
        claves = claves_menu()
        sel = min(sel, len(claves) - 1)
        dibujar_menu(claves, sel)
        ev = esperar_boton()
        if ev == "var":
            sel = (sel + 1) % len(claves)
        elif ev == "var_largo":
            sel = (sel - 1) % len(claves)
        elif ev == "sel":
            if claves[sel] == "ini":
                return
            editar(claves[sel])


# ================= Modo MENSAJES - transmisor =================
def pantalla_tx(msg, cont, estado, hint="12L: menu"):
    t = cont["ok"] + cont["err"]
    txt = msg.decode()
    mostrar(cabecera("TX") + [txt[:16], txt[16:32],
            "OK:%d E:%d %d%%" % (cont["ok"], cont["err"], 100 * cont["err"] // t if t else 0),
            estado, hint])

def enviar_uno(msg, cont):
    """Envia un mensaje, actualiza contadores y LEDs. Devuelve True si hubo ACK."""
    leds(a=1)
    ok = nrf.send(msg)
    print("TX:", msg, "OK" if ok else "SIN ACK", "reintentos:", nrf.reintentos)
    if ok:
        cont["ok"] += 1
        leds(v=1)
    else:
        cont["err"] += 1
        leds(r=1)
    return ok

def loop_tx(cont):
    """Envia en bucle. Cualquier boton lo detiene; 12L ademas vuelve al menu."""
    espera = INTERVALOS[cfg["iv"]]
    i = cfg["msg"]
    n = 0
    while True:
        msg = MENSAJES[i] if cfg["env"] == 2 else MENSAJES[cfg["msg"]]
        n += 1
        enviar_uno(msg, cont)
        pantalla_tx(msg, cont, "Loop #%d (R:%d)" % (n, nrf.reintentos), "Boton: parar")
        t0 = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), t0) < espera:
            ev = leer_botones()
            if ev:
                leds(a=1)
                return "menu" if ev == "sel_largo" else None
            time.sleep_ms(5)
        if cfg["env"] == 2:
            i = (i + 1) % len(MENSAJES)

def tx_continuo(msg, cont):
    """Paquetes uno tras otro sin parar (ciclo de trabajo 100 %), sin ACK.
    Sirve para medir frecuencia, ancho de banda y espectrograma (punto 4.4)."""
    nrf.iniciar_continuo(msg)
    leds(v=1)
    pantalla_tx(msg, cont, "TX CONTINUO", "Boton: parar")
    ev = esperar_boton()
    nrf.detener_continuo()
    leds(a=1)
    return "menu" if ev == "sel_largo" else None

def correr_tx():
    aplicar_radio(ack=(cfg["env"] != 3), crc=True)
    cont = {"ok": 0, "err": 0}
    estado = "12:enviar 13:msg"
    leds(a=1)
    while True:
        msg = MENSAJES[cfg["msg"]]
        pantalla_tx(msg, cont, estado)
        ev = esperar_boton()
        if ev == "sel_largo":                        # atras
            return
        if ev == "var":                              # siguiente mensaje
            cfg["msg"] = (cfg["msg"] + 1) % len(MENSAJES)
            estado = "Msg %d/%d" % (cfg["msg"] + 1, len(MENSAJES))
        elif ev == "sel":
            if cfg["env"] == 0:                      # envio unico
                pantalla_tx(msg, cont, "Enviando...")
                ok = enviar_uno(msg, cont)
                estado = "Enviado" if ok else "ERROR sin ACK"
                pantalla_tx(msg, cont, estado)
                time.sleep_ms(800)
                leds(a=1)
            elif cfg["env"] == 3:
                if tx_continuo(msg, cont) == "menu":
                    return
                estado = "Continuo parado"
            else:
                if loop_tx(cont) == "menu":
                    return
                estado = "Loop detenido"


# ================= Modo MENSAJES - receptor =================
def imprimible(data):
    return "".join(chr(c) if 32 <= c < 127 else "." for c in data)

def correr_rx():
    aplicar_radio(ack=True, crc=True)
    nrf.start_rx()
    n = 0
    texto = ""
    estado = "Esperando..."
    t_led = 0
    leds(a=1)
    while True:
        mostrar(cabecera("RX") + [texto[:16], texto[16:32], "Recibidos: %d" % n,
                                  estado, "12:cero 12L: menu"])
        pendiente = True
        while pendiente:
            if nrf.available():
                data = nrf.recv().rstrip(b'\x00')
                texto = imprimible(data)
                n += 1
                print("RX:", texto)
                leds(v=1)
                t_led = time.ticks_add(time.ticks_ms(), 600)
                estado = "Recibido"
                pendiente = False
            if t_led and time.ticks_diff(time.ticks_ms(), t_led) >= 0:
                leds(a=1)
                t_led = 0
            ev = leer_botones()
            if ev == "sel_largo":                    # atras
                return
            if ev == "sel":                          # reiniciar contador
                n = 0
                texto = ""
                estado = "Esperando..."
                pendiente = False
            time.sleep_ms(5)


# ================= Punto 4.3: prueba de BER =================
def gen_alfabetos():
    base = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789"
    return (base * 40)[:N_ARCHIVO].encode()

def gen_aleatorio():
    """Caracteres ASCII pseudoaleatorios (LCG con semilla fija: igual en las dos Pico)."""
    x = 12345
    out = bytearray(N_ARCHIVO)
    for i in range(N_ARCHIVO):
        x = (x * 1103515245 + 12345) & 0x7FFFFFFF
        out[i] = 33 + ((x >> 16) % 94)
    return bytes(out)

ARCHIVOS = [None, ("alfabetos.txt", gen_alfabetos), ("aleatorio.txt", gen_aleatorio)]

def cargar_archivo():
    """Lee el txt de la Pico. Si no existe lo crea (contenido identico en las dos Pico).
    Tambien se puede copiar un txt propio con Thonny, pero debe ser el MISMO en TX y RX."""
    nombre, gen = ARCHIVOS[cfg["prueba"]]
    datos = b""
    try:
        with open(nombre, "rb") as f:
            datos = f.read()
    except OSError:
        pass
    if not datos:
        datos = gen()
        with open(nombre, "wb") as f:
            f.write(datos)
    return nombre, datos

def checksum(datos):
    return sum(datos) & 0xFFFF

def chunk(arch, idx):
    """Trozo idx del archivo (28 bytes, relleno con ceros al final)."""
    c = arch[idx * DATOS_BER:(idx + 1) * DATOS_BER]
    return c + bytes(DATOS_BER - len(c))

def armar_trama(seq, datos):
    h, l = (seq >> 8) & 0xFF, seq & 0xFF
    return bytes([h, l, ~h & 0xFF, ~l & 0xFF]) + datos

def armar_fin(total, pas, chk):
    return FIN_HDR + bytes([(total >> 8) & 0xFF, total & 0xFF, pas,
                            (chk >> 8) & 0xFF, chk & 0xFF])

def ber_valor(est):
    return est["err"] / est["bits"] if est["bits"] else 0.0

def ber_nuevo():
    return {"ok": 0, "hdr": 0, "perd": 0, "bits": 0, "err": 0, "ult": None,
            "fin": False, "total": 0, "chk_tx": 0}

def ber_procesar(est, data, arch, n_chunks):
    """Procesa un paquete recibido. Devuelve 'fin' cuando llega la trama de fin."""
    if data[0:4] == FIN_HDR:
        if est["fin"]:
            return None                              # copia repetida de la trama de fin
        est["fin"] = True
        est["total"] = (data[4] << 8) | data[5]
        est["chk_tx"] = (data[7] << 8) | data[8]
        return "fin"
    if data[2] != (~data[0] & 0xFF) or data[3] != (~data[1] & 0xFF):
        if not est["fin"]:
            est["hdr"] += 1                          # encabezado danado: paquete no usable
        return None
    if est["fin"]:                                   # llego un paquete nuevo: otra prueba
        est.update(ber_nuevo())
    seq = (data[0] << 8) | data[1]
    if est["ult"] is not None:
        gap = (seq - est["ult"] - 1) & 0xFFFF
        if 0 < gap < 30000:
            est["perd"] += gap                       # paquetes que nunca llegaron
    est["ult"] = seq
    esperado = chunk(arch, seq % n_chunks)
    est["err"] += sum(POP[a ^ b] for a, b in zip(data[4:4 + DATOS_BER], esperado))
    est["bits"] += DATOS_BER * 8
    est["ok"] += 1
    return None

def guardar_resultado(est, nombre, chk_local):
    total = est["total"]
    per = (total - est["ok"]) / total if total else 0
    linea = "%s,%d,%s,%d,%d,%d,%d,%d,%d,%.3e,%.4f,%s\n" % (
        nombre, cfg["can"], cfg["vel"], cfg["lna"], total, est["ok"], est["hdr"],
        est["bits"], est["err"], ber_valor(est), per,
        "ok" if chk_local == est["chk_tx"] else "ARCHIVO_DISTINTO")
    try:
        os.stat(ARCHIVO_CSV)
    except OSError:
        with open(ARCHIVO_CSV, "w") as f:
            f.write("archivo,canal,vel,lna,paq_tx,paq_rx,hdr_malos,bits,bits_err,BER,PER,chk\n")
    with open(ARCHIVO_CSV, "a") as f:
        f.write(linea)
    print("RESULTADO BER:", linea.strip())

def pantalla_ber_tx(nombre, n_chunks, pas, chk, progreso, estado, hint):
    mostrar(cabecera("TX-BER", True) + [nombre, "%dB %dpkt x%d" % (
        n_chunks * DATOS_BER, n_chunks, pas), "chk:%04X" % chk, progreso if progreso else estado, hint])

def correr_ber_tx():
    nombre, arch = cargar_archivo()
    n_chunks = (len(arch) + DATOS_BER - 1) // DATOS_BER
    pas = PASADAS[cfg["pas"]]
    total = min(n_chunks * pas, 65535)
    chk = checksum(arch)
    print("BER TX:", nombre, len(arch), "bytes,", n_chunks, "paquetes x", pas)
    aplicar_radio(ack=False, crc=False)              # sin ACK y sin CRC: asi se ven los errores
    leds(a=1)
    while True:
        pantalla_ber_tx(nombre, n_chunks, pas, chk, "", "12: enviar", "12L: menu")
        ev = esperar_boton()
        if ev == "sel_largo":
            return
        if ev != "sel":
            continue
        enviados = 0
        for seq in range(total):
            nrf.send(armar_trama(seq, chunk(arch, seq % n_chunks)))
            enviados = seq + 1
            leds(v=seq & 1)                          # el LED verde parpadea al transmitir
            if seq % 16 == 0:
                pantalla_ber_tx(nombre, n_chunks, pas, chk,
                                "Env %d/%d" % (enviados, total), "", "Boton: abortar")
            time.sleep_ms(PAUSA_BER_MS)
            if hay_boton():
                esperar_soltar()
                break
        fin = armar_fin(enviados, pas, chk)
        for _ in range(5):                           # la trama de fin se repite (no hay ACK)
            nrf.send(fin)
            time.sleep_ms(20)
        print("BER TX enviados:", enviados)
        leds(a=1)
        pantalla_ber_tx(nombre, n_chunks, pas, chk, "", "Enviados: %d" % enviados, "12:otra 12L:menu")

def pantalla_ber_rx(est, estado):
    mostrar(["RX-BER Ch%d %s" % (cfg["can"], estado),
             "%dMHz %s" % (2400 + cfg["can"], VEL_TXT[cfg["vel"]]),
             "Sens:%sdBm" % fmt_num(sens_dbm()),
             "Rx:%d Perd:%d" % (est["ok"], est["perd"]),
             "Bits:%d" % est["bits"],
             "ErrBits:%d" % est["err"],
             "BER:%.2e" % ber_valor(est),
             "12:cero 12L: menu"])

def correr_rx_ber():
    nombre, arch = cargar_archivo()
    n_chunks = (len(arch) + DATOS_BER - 1) // DATOS_BER
    chk = checksum(arch)
    print("BER RX:", nombre, len(arch), "bytes, chk", chk)
    aplicar_radio(ack=False, crc=False)
    nrf.start_rx()
    est = ber_nuevo()
    estado = "..."
    t_pant = time.ticks_ms()
    t_led = 0
    sucio = True
    leds(a=1)
    while True:
        if nrf.available():
            r = ber_procesar(est, nrf.recv(), arch, n_chunks)
            leds(v=1)
            t_led = time.ticks_add(time.ticks_ms(), 300)
            sucio = True
            if r == "fin":
                guardar_resultado(est, nombre, chk)
                estado = "FIN" if chk == est["chk_tx"] else "FIN!chk"
            elif est["ok"] and not est["fin"]:
                estado = "RX"
        elif sucio and time.ticks_diff(time.ticks_ms(), t_pant) > 1000:
            pantalla_ber_rx(est, estado)             # el OLED tarda ~25 ms: se redibuja poco
            t_pant = time.ticks_ms()
            sucio = False
        if t_led and time.ticks_diff(time.ticks_ms(), t_led) >= 0:
            leds(a=1)
            t_led = 0
            pantalla_ber_rx(est, estado)
            t_pant = time.ticks_ms()
            sucio = False
        ev = leer_botones()
        if ev == "sel_largo":
            return
        if ev == "sel":
            est.update(ber_nuevo())
            estado = "..."
            sucio = True


# ================= Programa principal =================
def principal():
    inicio()
    while True:
        try:
            menu()
            if cfg["rol"] == 0:
                correr_tx() if cfg["prueba"] == 0 else correr_ber_tx()
            else:
                correr_rx() if cfg["prueba"] == 0 else correr_rx_ber()
        except Exception as e:                       # un error vuelve al menu, no hay que reiniciar
            print("ERROR:", repr(e))
            leds(r=1)
            mostrar(["ERROR:", str(e)[:16], "", "Volviendo al menu"])
            time.sleep_ms(2500)
        nrf.apagar()
        leds(a=1)

if __name__ == "__main__":
    principal()