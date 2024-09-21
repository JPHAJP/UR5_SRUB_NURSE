// Función para enviar el formulario al cambiar el estado del interruptor o imán
function enviarFormulario() {
    document.getElementById("controlForm").submit();
}

// Función para cambiar el color del botón y mantener el estado seleccionado
function toggleButtonColor(button) {
    button.classList.toggle('bg-blue-500');  // Color predeterminado
    button.classList.toggle('bg-green-500'); // Color al hacer clic
    button.classList.toggle('text-white');   // Cambiar el color del texto si es necesario
}

function changeColorTemporary(button) {
    // Cambia el color del botón al hacer clic
    button.classList.add('bg-green-500');
    button.classList.remove('bg-blue-500');

    // Después de 300ms, vuelve al color original
    setTimeout(function() {
        button.classList.remove('bg-green-500');
        button.classList.add('bg-blue-500');
    }, 400);  // 300ms de espera antes de volver al color original
}
