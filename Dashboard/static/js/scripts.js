// Función para enviar el formulario al cambiar el estado del checkbox
function enviarFormulario() {
    document.getElementById("controlForm").submit();
}

// Función para manejar el cambio de estado de los checkboxes
function handleCheckboxChange(checkbox) {
    const toggleDot = checkbox.nextElementSibling.querySelector('.toggle-dot');
    if (checkbox.checked) {
        toggleDot.classList.add('translate-x-4');
        toggleDot.parentElement.classList.add('bg-[var(--color-accent)]');
    } else {
        toggleDot.classList.remove('translate-x-4');
        toggleDot.parentElement.classList.remove('bg-[var(--color-accent)]');
    }
    enviarFormulario();
}

// Función para inicializar los eventos de los checkboxes
function initializeCheckboxes() {
    const checkboxes = document.querySelectorAll('input[type="checkbox"]');
    checkboxes.forEach(checkbox => {
        checkbox.addEventListener('change', function() {
            handleCheckboxChange(this);
        });
    });
}

// Función para cambiar temporalmente el color del botón
function changeColorTemporary(button) {
    const originalColor = button.classList.contains('bg-[var(--color-primary)]') 
        ? 'bg-[var(--color-primary)]' 
        : 'bg-[var(--color-secondary)]';
    
    button.classList.remove(originalColor);
    button.classList.add('bg-[var(--color-accent)]');

    setTimeout(function() {
        button.classList.remove('bg-[var(--color-accent)]');
        button.classList.add(originalColor);
    }, 400);
}

// Función para inicializar los eventos de los botones
function initializeButtons() {
    const buttons = document.querySelectorAll('button');
    buttons.forEach(button => {
        button.addEventListener('click', function() {
            changeColorTemporary(this);
        });
    });
}

// Inicializar los checkboxes y botones cuando el DOM esté cargado
document.addEventListener('DOMContentLoaded', () => {
    initializeCheckboxes();
    initializeButtons();
});

document.addEventListener('DOMContentLoaded', () => {
    const interruptorCheckbox = document.getElementById('interruptor-checkbox');
    const imanCheckbox = document.getElementById('iman-checkbox');
    
    // Ejemplo de función que podrías ejecutar al cambiar el checkbox
    interruptorCheckbox.addEventListener('change', function() {
        if (this.checked) {
            console.log('Interruptor activado');
        } else {
            console.log('Interruptor desactivado');
        }
    });

    imanCheckbox.addEventListener('change', function() {
        if (this.checked) {
            console.log('Imán activado');
        } else {
            console.log('Imán desactivado');
        }
    });
});
