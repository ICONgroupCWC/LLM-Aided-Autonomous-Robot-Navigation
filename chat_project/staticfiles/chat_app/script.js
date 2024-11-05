document.addEventListener("DOMContentLoaded", function() {
    const form = document.getElementById("message-form");
    const chatContainer = document.getElementById("chat-container");

    form.addEventListener("submit", function(event) {
        event.preventDefault();
        const userInput = document.getElementById("user-input").value.trim();
        if (userInput === "") return;
        addMessage(userInput, "user-message");
        document.getElementById("user-input").value = "";
        sendMessage(userInput);
    });

    function addMessage(message, className) {
        const messageDiv = document.createElement("div");
        messageDiv.textContent = message;
        messageDiv.classList.add("message", className);
        chatContainer.appendChild(messageDiv);
        chatContainer.scrollTop = chatContainer.scrollHeight;
    }

    function sendMessage(message) {
        fetch("/send_message", {
            method: "POST",
            body: JSON.stringify({ message: message }),
            headers: {
                "Content-Type": "application/json",
                "X-CSRFToken": getCookie("csrftoken") // Include the CSRF token
            }
        })
        .then(response => response.json())
        .then(data => addMessage(data.message, "bot-message"))
        .catch(error => console.error('Error:', error));
    }

    // Function to get the CSRF token from cookies
    function getCookie(name) {
        var cookieValue = null;
        if (document.cookie && document.cookie !== '') {
            var cookies = document.cookie.split(';');
            for (var i = 0; i < cookies.length; i++) {
                var cookie = cookies[i].trim();
                if (cookie.substring(0, name.length + 1) === (name + '=')) {
                    cookieValue = decodeURIComponent(cookie.substring(name.length + 1));
                    break;
                }
            }
        }
        return cookieValue;
    }
});
