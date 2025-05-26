# Control por Rechazo Activo de Perturbaciones (ADRC)

El Control por Rechazo Activo de Perturbaciones (ADRC, por sus siglas en inglés *Active Disturbance Rejection Control*) es una técnica avanzada de control que surge como una alternativa robusta al control PID tradicional. Desarrollado por el Dr. Zhiqiang Gao, el ADRC busca mitigar las limitaciones del PID, especialmente su dependencia de un modelo preciso de la planta y su sensibilidad a perturbaciones externas e incertidumbres internas. 

El ADRC se basa en tres pilares fundamentales:
1. **Estimación de perturbaciones**: Utiliza un Observador de Estados Extendido (ESO) para estimar y compensar perturbaciones en tiempo real.
2. **Independencia del modelo**: Requiere solo información básica del sistema, como su orden y ganancia nominal.
3. **Simplificación del diseño**: Transforma el sistema real en una planta nominal de fácil control.

Esta técnica ha demostrado ser efectiva en aplicaciones de control de movimiento y procesos industriales, donde las perturbaciones y variaciones paramétricas son comunes.

---
## Definiciones

> 🔑 **_ADRC (Active Disturbance Rejection Control)_**: Estrategia de control robusto que estima y cancela activamente perturbaciones mediante un observador de estados extendido (ESO), requiriendo mínimo conocimiento del modelo de la planta. Combina realimentación no lineal y compensación en tiempo real de incertidumbres.

> 🔑 **_Observador de Estados Extendido (ESO)_**: Componente clave del ADRC que expande el espacio de estados para incluir la "perturbación total" como estado adicional, permitiendo su estimación y posterior cancelación. Opera bajo el principio de que cualquier desviación del modelo nominal puede tratarse como perturbación.

> 🔑 **_Perturbación Total (ε(t))_**: Término agregado que engloba dinámicas no modeladas, variaciones paramétricas y perturbaciones externas. El ESO estima ε(t) como un estado extendido, simplificando el diseño del controlador.

## Componentes del ADRC

El **Control por Rechazo Activo de Perturbaciones (ADRC)** se estructura en tres componentes fundamentales que trabajan sinérgicamente para lograr un control robusto independiente del modelo preciso de la planta. El **Generador de Trayectorias** transforma referencias abruptas en perfiles suaves, preservando los actuadores de esfuerzos bruscos. El **Observador de Estados Extendido (ESO)**, corazón del ADRC, estima en tiempo real tanto los estados no medibles como las perturbaciones totales (internas y externas), agrupándolas en una única señal a compensar. Finalmente, la **Ley de Control** combina realimentación de estados y cancelación activa de perturbaciones, simplificando el sistema a una cadena de integradores nominales. Esta arquitectura permite controlar sistemas complejos con sólo conocer su orden dinámico y una aproximación gruesa de su ganancia, demostrando especial eficacia en sistemas no lineales, de parámetros variables o bajo perturbaciones significativas, superando así limitaciones clásicas del control PID tradicional.

### 1. Generador de Trayectorias
Define la referencia deseada para el sistema. Ejemplo:

Para un sistema de segundo orden, el generador puede ser:  

#### 1.1 Formulación Matemática
```math
\begin{aligned}
&\text{Sistema de 2do orden:} \\
&\ddot{y}^* + 2\zeta\omega_n\dot{y}^* + \omega_n^2y^* = \omega_n^2r(t) \\
&\text{Donde:} \\
&\quad \omega_n = \text{Frecuencia natural} \\
&\quad \zeta = \text{Factor de amortiguamiento}
\end{aligned}
```

#### 1.1 Implementación en Python

```math
import numpy as np

def generate_trajectory(t, r, wn, zeta):
    """Genera trayectoria suavizada para referencia escalón"""
    y = np.zeros_like(t)
    yd = np.zeros_like(t)
    ydd = np.zeros_like(t)
    
    for i in range(1, len(t)):
        dt = t[i] - t[i-1]
        ydd[i] = wn**2*(r[i] - y[i-1]) - 2*zeta*wn*yd[i-1]
        yd[i] = yd[i-1] + ydd[i]*dt
        y[i] = y[i-1] + yd[i]*dt
        
    return y, yd, ydd
```

#### 1.2 Ejemplo de Uso

```math
import matplotlib.pyplot as plt

t = np.linspace(0, 5, 500)
r = np.ones_like(t)   Referencia escalón
y, yd, ydd = generate_trajectory(t, r, wn=2.0, zeta=0.7)

plt.figure(figsize=(10,6))
plt.plot(t, r, 'r--', label='Referencia')
plt.plot(t, y, 'b-', label='Trayectoria generada')
plt.legend(); plt.grid(True)
plt.title('Generador de Trayectorias ADRC')
plt.show()
```

### 2. Observador de Estados Extendido (ESO)


![image](https://github.com/user-attachments/assets/eb9b9ea5-5c38-4470-96d9-393d47daef4c)

fig 2 . Observador de Estados Extendido


El **Observador de Estados Extendido (ESO** es el componente central del control ADRC que permite estimar y compensar perturbaciones en tiempo real sin requerir un modelo preciso del sistema. Su funcionamiento se basa en expandir el espacio de estados tradicional para incluir no solo las variables de estado físicas, sino también un "estado extendido" que representa la perturbación total que afecta al sistema (incluyendo dinámicas no modeladas, perturbaciones externas y variaciones paramétricas).

El ESO opera mediante un conjunto de ecuaciones diferenciales acopladas que comparan continuamente la salida real del sistema con la salida estimada, ajustando dinámicamente sus estados internos. Para un sistema de segundo orden, por ejemplo, utiliza tres ecuaciones: dos para estimar los estados físicos (posición y velocidad) y una tercera para estimar la perturbación agregada. Las ganancias del observador (β) se sintonizan típicamente en función de un único parámetro (el ancho de banda ωₒ), lo que simplifica notablemente su implementación práctica.

La verdadera potencia del ESO radica en su capacidad para transformar virtualmente cualquier sistema en una cadena simple de integradores, independientemente de sus no linealidades o perturbaciones. Esto se logra estimando la perturbación total en tiempo real y cancelándola directamente en la ley de control. Como resultado, el controlador solo necesita manejar la dinámica nominal simplificada, haciendo que el sistema en lazo cerrado sea inherentemente robusto y fácil de sintonizar.

#### 2.1 Implementación

Estima estados no medibles y perturbaciones. Para un sistema lineal:  

```math
\begin{cases}
\dot{z}_1 = z_2 + \beta_1(y - z_1) \\
\dot{z}_2 = z_3 + \beta_2(y - z_1) + b_0u \\
\dot{z}_3 = \beta_3(y - z_1)
\end{cases}
```

#### 2.2 Demostracion en MATLAB

```math
% MATLAB: Cálculo de ganancias para ancho de banda wo
n = 2;  % Orden del sistema
wo = 30; % Frecuencia del observador
beta = zeros(n+1,1);
for i = 1:n+1
    beta(i) = factorial(n+1)/(factorial(i)*factorial(n+1-i)) * wo^i;
end
```
# 💡  Ejemplos de Observador de Estados Extendido (ESO)

## 💡  Ejemplo 1: ESO para un sistema lineal de segundo orden (tiempo continuo)

Consideremos un sistema lineal:

$$
\ddot{x}(t) + a_1 \dot{x}(t) + a_0 x(t) = b \cdot u(t)
$$

Se puede escribir como sistema de estado:

$$
\begin{cases}
\dot{x}_1 = x_2 \\
\dot{x}_2 = -a_1 x_2 - a_0 x_1 + b u(t) + d(t)
\end{cases}
$$

El ESO estima $x_1$, $x_2$ y la perturbación $d(t)$:

$$
\begin{cases}
\dot{z}_1 = z_2 + L_1 \cdot (y - z_1) \\
\dot{z}_2 = -a_1 z_2 - a_0 z_1 + b u(t) + z_3 + L_2 \cdot (y - z_1) \\
\dot{z}_3 = L_3 \cdot (y - z_1)
\end{cases}
$$

Donde:

- $z_1 \approx x_1$
- $z_2 \approx x_2$
- $z_3 \approx d(t)$
- $e(t) = y(t) - z_1(t)$

---

## 💡  Ejemplo 2: ESO en tiempo discreto (Euler)

# Parámetros del sistema
$$
a0 = 2
a1 = 3
b = 1
Ts = 0.001
omega = 30
$$

# Ganancias del ESO
$$
L1 = 3 * omega
L2 = 3 * omega**2
L3 = omega**3
$$

# Inicialización de estados
$$
z1 = 0
z2 = 0
z3 = 0
$$

```python
def ESO_discrete_step(y, u):
    global z1, z2, z3
    e = y - z1
    dz1 = z2 + L1 * e
    dz2 = -a1*z2 - a0*z1 + b*u + z3 + L2 * e
    dz3 = L3 * e

    z1 += Ts * dz1
    z2 += Ts * dz2
    z3 += Ts * dz3

    return z1, z2, z3
```

### 3. Ley de Control del ADRC


![image](https://github.com/user-attachments/assets/667b5e12-d0b5-46b6-9b07-9c72ce3b227e)

Fig 3. Ley de Control del ADRC.


El tercer pilar del ADRC es su innovadora **Ley de Control**, que combina de manera elegante la realimentación de estados con la cancelación activa de perturbaciones. A diferencia de los esquemas de control tradicionales que intentan rechazar perturbaciones después de que afectan al sistema, el ADRC las anticipa y neutraliza mediante un enfoque de dos etapas:

1. **Compensación de Perturbaciones**: Utiliza la estimación de la perturbación total (zₙ₊₁) proporcionada por el ESO para cancelarla directamente en la señal de control.
2. **Realimentación de Estados**: Aplica una ley de control convencional (usualmente PD) sobre el sistema nominal libre de perturbaciones.

La forma general de la ley es:

$u = \frac{u_0 - z_{n+1}}{b_0}$

## Ejercicios Resueltos

### Ejercicio 1: Diseño de un ADRC para un sistema masa-resorte-amortiguador

**Planta**:  
$M\ddot{y} + B\dot{y} + Ky = u(t) + w(t)$

**Paso 1**: Reformular como:  

$\ddot{y} = \frac{u}{M} + \epsilon(t), \quad \epsilon(t) = -\frac{B}{M}\dot{y} - \frac{K}{M}y + w(t)$
**Paso 2**: Diseñar ESO para estimar \( \epsilon(t) \).  
**Paso 3**: Implementar la ley de control con polos en \( s = -15 \) (críticamente amortiguado).

---

### 4 Explicación del Tema: Rechazo Activo a Perturbaciones en Sistemas No Lineales  

![image](https://github.com/user-attachments/assets/35f85db7-d293-4b7c-bd41-e8346f045f29)

Fig 4. Explicación del Tema: Rechazo Activo a Perturbaciones en Sistemas No Lineales .


El **Rechazo Activo a Perturbaciones (RAP)** es una técnica de control avanzada diseñada para mitigar o eliminar los efectos de perturbaciones externas o internas en sistemas dinámicos, especialmente en aquellos que son **no lineales**.  

---

#### **Conceptos Clave**  
1. **Perturbaciones**:  
   - Señales o fuerzas externas no deseadas (ej: ruido, variaciones de carga).  
2. **Sistemas No Lineales**:  
   - Sistemas cuya respuesta no es proporcional a la entrada (ej: robots, motores eléctricos).  
3. **Rechazo Activo**:  
   - Detección y compensación en tiempo real de perturbaciones.  

---

#### **Objetivos del RAP**  
| Objetivo          | Descripción                                                                 |  
|-------------------|-----------------------------------------------------------------------------|  
| **Estabilidad**   | Mantener el sistema estable ante perturbaciones.                            |  
| **Robustez**      | Funcionamiento correcto incluso con modelos imperfectos del sistema.        |  
| **Precisión**     | Minimizar errores causados por perturbaciones.                              |  

---

#### **Técnicas Comunes**  
- Control por Modos Deslizantes (SMC): Ideal para sistemas no lineales, forza al sistema a seguir una trayectoria deseada a pesar de perturbaciones.

- Observadores de Perturbaciones: Estimán y compensan perturbaciones en tiempo real (ej: Observador de Estados Extendido - ESO).

- Control Adaptativo: Ajusta parámetros del controlador según cambios en el sistema o perturbaciones.


### 5 Rechazo Activo a Perturbaciones en Sistemas Lineales

El **Rechazo Activo a Perturbaciones (RAP)** en sistemas lineales es una estrategia fundamental en teoría de control para mantener el desempeño del sistema ante influencias externas no deseadas. A diferencia de los sistemas no lineales, aquí se aprovecha la superposición y las propiedades de linealidad para diseñar compensadores eficientes.

---

#### Conceptos Básicos
#### 1. Perturbaciones en Sistemas Lineales
- **Definición**: Señales externas o variaciones de parámetros que afectan la salida (ej: ruido eléctrico, vibraciones mecánicas).
- **Clasificación**:
  - **Aditivas**: `y(t) = G(s)u(t) + d(t)`
  - **Multiplicativas**: Cambios en parámetros del sistema (ej: resistencia en circuitos).

#### 2. Propiedades Clave
- **Superposición**: La respuesta a múltiples perturbaciones es la suma de sus efectos individuales.
- **Invariante en el Tiempo**: Comportamiento predecible ante perturbaciones constantes.

---

#### Técnicas de Rechazo
#### A. Control Integral (I) y PID
```math
u(t) = K_p e(t) + K_i \int e(t)dt + K_d \frac{de(t)}{dt}
```









### 6 Observador de Luenberger

En sistemas de control, no siempre es posible medir todas las variables de estado del sistema. Para estimar estas variables no medibles, se utilizan **observadores**. Uno de los más comunes y fundamentales es el **Observador de Luenberger**, diseñado para sistemas lineales.

Este observador reconstruye el estado del sistema a partir de la salida medida y la entrada conocida, permitiendo así implementar técnicas de control de estado incluso cuando no se dispone de todos los estados del sistema.

---

#### Representación del Sistema

Consideremos un sistema lineal invariante en el tiempo representado en espacio de estados:

```math
\[
\begin{aligned}
\dot{x}(t) &= Ax(t) + Bu(t) \\
y(t) &= Cx(t)
\end{aligned}
\]

```

donde:
- $x(t) \in \mathbb{R}^n \)$ es el vector de estado,
- $u(t) \in \mathbb{R}^m \)$ es la entrada,
- $y(t) \in \mathbb{R}^p \)$ es la salida,
- $( A, B, C \)$ son matrices de dimensiones apropiadas.

---

#### Observador de Luenberger

El observador de Luenberger estima el estado \( \hat{x}(t) \) del sistema mediante:

```math
\[
\begin{aligned}
\dot{\hat{x}}(t) &= A\hat{x}(t) + Bu(t) + L(y(t) - \hat{y}(t)) \\
\hat{y}(t) &= C\hat{x}(t)
\end{aligned}
\]

```

donde:
- $\hat{x}(t)$ es el estado estimado,
- \( L \) es la **ganancia del observador**, diseñada para garantizar estabilidad y rapidez de la estimación.

---

#### Dinámica del Error

Definimos el **error de estimación** como:

$e(t) = x(t) - \hat{x}(t)$

Derivando:

```math
\[
\begin{aligned}
\dot{e}(t) &= \dot{x}(t) - \dot{\hat{x}}(t) \\
&= Ax(t) + Bu(t) - \left[A\hat{x}(t) + Bu(t) + L(y - \hat{y})\right] \\
&= A(x - \hat{x}) - L(Cx - C\hat{x}) \\
&= (A - LC)e(t)
\end{aligned}
\]

```

Por lo tanto, la **dinámica del error** depende de la matriz \( A - LC \). El observador será **estable** si los autovalores de \( A - LC \) están en el semiplano izquierdo del plano complejo (para sistemas continuos).

---

#### Ejercicio Resuelto

#### Sistema:
$$
\[
A = \begin{bmatrix} 0 & 1 \\ -2 & -3 \end{bmatrix}, \quad
B = \begin{bmatrix} 0 \\ 1 \end{bmatrix}, \quad
C = \begin{bmatrix} 1 & 0 \end{bmatrix}
\]$$

**Diseñar un observador de Luenberger** tal que los polos del observador estén en \( s = -5 \pm 1 \).

#### Paso 1: Confirmar observabilidad

Calculamos la matriz de observabilidad:
$$
\mathcal{O} = \begin{bmatrix} C \\ CA \end{bmatrix} = 
\begin{bmatrix} 1 & 0 \\ 0 & 1 \end{bmatrix}
$$

Es de rango completo \( \Rightarrow \) el sistema es observable.

---

### Paso 2: Colocar los polos del observador

Queremos que \( A - LC \) tenga los polos en \( s = -5 \pm 1 \), lo que corresponde al polinomio característico:

$(s + 5 - 1)(s + 5 + 1) = s^2 + 10s + 26$

Supongamos:

$L = \begin{bmatrix} l_1 \\ l_2 \end{bmatrix}$

Entonces:

$$
A - LC = \begin{bmatrix} 0 & 1 \\ -2 & -3 \end{bmatrix} - 
\begin{bmatrix} l_1 \\ l_2 \end{bmatrix} [1 \quad 0] =
\begin{bmatrix} -l_1 & 1 \\ -2 - l_2 & -3 \end{bmatrix}$$

Calculamos el polinomio característico de \( A - LC \):

$$
\det(sI - (A - LC)) = \left| 
\begin{bmatrix} s + l_1 & -1 \\ 2 + l_2 & s + 3 \end{bmatrix}
\right| = (s + l_1)(s + 3) + (2 + l_2)$$

$= s^2 + (l_1 + 3)s + (3l_1 + 2 + l_2)$

Igualamos con \( s^2 + 10s + 26 \):


$l_1 + 3 = 10 \Rightarrow l_1 = 7$
$3l_1 + 2 + l_2 = 26 \Rightarrow 21 + 2 + l_2 = 26 \Rightarrow l_2 = 3$

---

#### Resultado:

$L = \begin{bmatrix} 7 \\ 3 \end{bmatrix}$

---

#### Ejercicio Propuesto

Dado el sistema:


$A = \begin{bmatrix} 1 & 2 \\ 0 & 3 \end{bmatrix}$

$B = \begin{bmatrix} 0 \\ 1 \end{bmatrix}$

$C = \begin{bmatrix} 1 & 1 \end{bmatrix}$

1. Verifica si el sistema es observable.  
2. Diseña un observador de Luenberger con polos en \( s = -2 \) y \( s = -4 \).  
3. Calcula la matriz \( L \).  

*Tip:* Usa el método de asignación de polos mediante comparación de polinomios característicos.























## Ejercicios Adicionales

### Ejercicio 3: Sistema no lineal con acoplamiento trigonométrico

Dado el sistema:  
$\dot{x} = -k_1 x - k_2 \sin(x) + c u$  

#### 1. Linealización alrededor del punto de operación

**Paso 1:** Definir punto de operación  

Sea $(x_0, u_0)$ tal que $\dot{x} = 0$:  
$0 = -k_1 x_0 - k_2 \sin(x_0) + c u_0$

**Paso 2:** Expansión en serie de Taylor  

$\Delta \dot{x} \approx \left.\frac{\partial f}{\partial x}\right|_{x_0} \Delta x + \left.\frac{\partial f}{\partial u}\right|_{u_0} \Delta u=$  

Donde:  
$\frac{\partial f}{\partial x} = -k_1 - k_2 \cos(x), \quad \frac{\partial f}{\partial u} = c$

**Resultado linealizado:**  

$\Delta \dot{x} = (-k_1 - k_2 \cos(x_0)) \Delta x + c \Delta u$

#### 2. Diseño de NADRC con función `fal(e, α, δ)`

**Estructura del controlador:**  
```python
def fal(e, alpha, delta):
    return np.abs(e)**alpha * np.sign(e) if np.abs(e) > delta else e/(delta**(1-alpha))
```

# Parámetros NADRC
alpha = 0.5  # Factor no lineal
delta = 0.1  # Zona lineal
b0 = c       # Ganancia aproximada

### Ejercicio 3: Variación paramétrica
Simular un ADRC para:  

$\ddot{y} = (4.75 - 4.5y)u + 0.7\dot{y} - 0.25y$
Evaluar desempeño con perturbaciones en rampa y sinusoidal.

---

## Conclusiones

- El control por Rechazo Activo de Perturbaciones (ADRC) ha demostrado ser una estrategia efectiva y robusta frente a incertidumbres del modelo y perturbaciones externas. A diferencia de los controladores PID tradicionales, cuya eficacia puede verse comprometida por variaciones en la dinámica del sistema, el ADRC permite una mejor adaptación y rendimiento en entornos cambiantes o poco conocidos, lo que lo convierte en una alternativa más confiable para sistemas complejos y de alta exigencia.

- Una de las ventajas más significativas del ADRC es que su implementación práctica requiere la sintonización de un menor número de parámetros en comparación con los controladores clásicos. Esto simplifica el proceso de diseño y ajuste, reduciendo el tiempo de puesta en marcha del sistema y minimizando el riesgo de errores asociados a una configuración inadecuada. Además, su estructura modular facilita la personalización según las necesidades específicas del sistema a controlar.

- El ADRC ha ganado terreno en diversas aplicaciones industriales debido a sus beneficios en cuanto a estabilidad y desempeño. Entre sus usos más destacados se encuentran el control de movimiento en sistemas robóticos, el control de velocidad y posición en motores eléctricos, así como el control de variables críticas en procesos químicos. Su capacidad para compensar perturbaciones de manera dinámica lo hace especialmente útil en entornos donde la precisión y la estabilidad son fundamentales para garantizar un funcionamiento óptimo y seguro.

- El ADRC se adapta eficientemente tanto a sistemas lineales como no lineales, gracias a su enfoque basado en observadores que permiten estimar dinámicamente las perturbaciones internas y externas del sistema, mejorando la precisión del control sin necesidad de un modelo matemático exacto.

- Este enfoque de control promueve una mayor estabilidad y rapidez en la respuesta transitoria del sistema, lo que lo hace especialmente útil en aplicaciones donde se requiere un alto desempeño dinámico, como el seguimiento de trayectorias y sistemas de tiempo real.

- El observador de estado extendido (ESO), elemento clave del ADRC, proporciona información crítica para estimar en tiempo real las perturbaciones totales, permitiendo una compensación inmediata que mejora la eficiencia energética y la vida útil de los actuadores en sistemas mecatrónicos.

- La filosofía del ADRC se basa en asumir que todo lo desconocido puede tratarse como una perturbación total, lo cual libera al diseñador del controlador de depender de modelos precisos y permite su uso en sistemas con alto grado de incertidumbre estructural o variaciones paramétricas.

- Su creciente adopción en sectores como la automatización industrial, vehículos autónomos, sistemas aeroespaciales y control de energía demuestra que el ADRC no solo es una herramienta académica, sino una solución con impacto real en la mejora del desempeño y confiabilidad de sistemas complejos.

---

## Referencias

1. **Gao, Z.** (2014). *Active Disturbance Rejection Control: A Paradigm Shift in Feedback Control Design*. IEEE.  
   Este artículo pionero introduce el ADRC como una nueva filosofía de diseño de controladores, enfatizando su capacidad para compensar perturbaciones sin requerir un modelo preciso del sistema.

2. **Universidad ECCI.** (2025). *Material del curso Control de Movimiento*.  
   Recurso didáctico esencial para comprender la implementación y análisis del ADRC en sistemas reales, con orientación práctica y contextualizada al entorno académico.

3. **Guo, B. Z., & Zhao, Z. L.** (2016). *On Convergence of Nonlinear Active Disturbance Rejection Control*. *SIAM Journal on Control and Optimization*.  
   Trabajo académico que respalda teóricamente la convergencia del ADRC en sistemas no lineales, aportando rigurosidad matemática al diseño del controlador.

4. **Han, J.** (2009). *From PID to Active Disturbance Rejection Control*. *IEEE Transactions on Industrial Electronics*.  
   Jianbo Han es uno de los investigadores más influyentes en el desarrollo del ADRC. Este artículo presenta la transición del enfoque clásico PID al ADRC, mostrando casos comparativos y argumentos sólidos para su adopción.

5. **Zhao, K., & Gao, Z.** (2012). *Nonlinear Control System Design Based on ADRC*. *Proceedings of the American Control Conference*.  
   Explora el diseño del ADRC aplicado a sistemas no lineales, con simulaciones y resultados experimentales que evidencian su eficacia frente a controladores tradicionales.

6. **Zhou, Y., & Han, J.** (2005). *A New Control Strategy: Active Disturbance Rejection Control (ADRC)*. *Control and Decision Conference*.  
   Esta obra temprana proporciona una introducción estructurada a la estrategia ADRC, definiendo su marco general y componentes fundamentales, como el Observador de Estado Extendido (ESO).

7. **Chen, W. H., Ballance, D. J., & Gawthrop, P. J.** (2000). *A Nonlinear Disturbance Observer for Robotic Manipulators*. *IEEE Transactions on Industrial Electronics*.  
   Aunque no aborda directamente el ADRC, presenta conceptos de observadores de perturbaciones que son esenciales en la arquitectura de ADRC, particularmente para sistemas robóticos.


