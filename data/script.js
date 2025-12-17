document.addEventListener('DOMContentLoaded', () => {
    // Views
    const mainMenu = document.getElementById('main-menu');
    const rcView = document.getElementById('rc-view');

    const settingsView = document.getElementById('settings-view');

    // Buttons
    const rcModeBtn = document.getElementById('rc-mode-btn');
    const loadMazeBtn = document.getElementById('load-maze-btn');
    const solveMazeBtn = document.getElementById('solve-maze-btn');
    const obstacleModeBtn = document.getElementById('obstacle-mode-btn');
    const settingsBtn = document.getElementById('settings-btn');
    const stopBtn = document.getElementById('stop-btn');
    const rcBackBtn = document.getElementById('rc-back-btn');
    const settingsBackBtn = document.getElementById('settings-back-btn');
    const powerOffBtn = document.getElementById('power-off-btn');
    const toggleModeBtn = document.getElementById('toggle-mode-btn');

    // Modals
    const loadMazeModal = document.getElementById('load-maze-modal');
    const solveMazeModal = document.getElementById('solve-maze-modal');
    const powerOffModal = document.getElementById('power-off-modal');

    // Modal components
    const mazeList = document.getElementById('maze-list');
    const loadSelectedMazeBtn = document.getElementById('load-selected-maze-btn');
    const mazeNameInput = document.getElementById('maze-name-input');
    const startSolvingBtn = document.getElementById('start-solving-btn');
    const abortSolvingBtn = document.getElementById('abort-solving-btn');
    const loader = document.querySelector('#solve-maze-modal .loader');
    const confirmPowerOffBtn = document.getElementById('confirm-power-off-btn');
    const cancelPowerOffBtn = document.getElementById('cancel-power-off-btn');

    // Close buttons
    const closeBtns = document.querySelectorAll('.close-btn');

    // --- WebSocket Communication ---
    let websocket;

    const initWebSocket = () => {
        // Replace with your ESP32's IP address
        websocket = new WebSocket('ws://' + window.location.hostname + ':81/');

        websocket.onopen = () => {
            console.log('WebSocket connection established');
        };

        websocket.onmessage = (event) => {
            console.log('Message from server: ', event.data);
            const [type, data] = event.data.split(/:(.*)/s);
            if (type === 'maze_solution') {
                const [name, solution] = data.split(';');
                saveMaze(name, solution);
                alert(`Maze "${name}" solved and saved!`);
                resetSolveMazeModal();
            } else if (type === 'maze_fail') {
                alert(`Failed to solve maze: ${data}`);
                resetSolveMazeModal();
            } else if (type === 'error_data') {
                addDataToChart(parseFloat(data));
            }
        };

        websocket.onclose = () => {
            console.log('WebSocket connection closed');
            // Optionally, try to reconnect
            setTimeout(initWebSocket, 2000);
        };

        websocket.onerror = (error) => {
            console.error('WebSocket error: ', error);
        };
    };

    const sendMessage = (message) => {
        if (websocket.readyState === WebSocket.OPEN) {
            websocket.send(message);
        } else {
            console.error('WebSocket is not connected.');
        }
    };

    // Initialize WebSocket on page load
    initWebSocket();

    const resetSolveMazeModal = () => {
        solveMazeModal.style.display = 'none';
        loader.classList.add('hidden');
        mazeNameInput.classList.remove('hidden');
        startSolvingBtn.classList.remove('hidden');
        abortSolvingBtn.classList.add('hidden');
        mazeNameInput.value = '';
    };

    // --- Maze Storage ---
    const getMazes = () => {
        const mazes = localStorage.getItem('mazes');
        return mazes ? JSON.parse(mazes) : [];
    };

    const saveMaze = (name, solution) => {
        const mazes = getMazes();
        mazes.push({ name, solution });
        localStorage.setItem('mazes', JSON.stringify(mazes));
    };

    const populateMazeList = () => {
        mazeList.innerHTML = '';
        const mazes = getMazes();
        mazes.forEach(maze => {
            const mazeItem = document.createElement('div');
            mazeItem.classList.add('maze-item');
            mazeItem.textContent = maze.name;
            mazeItem.addEventListener('click', () => {
                const currentSelected = document.querySelector('.maze-item.selected');
                if (currentSelected) {
                    currentSelected.classList.remove('selected');
                }
                mazeItem.classList.add('selected');
            });
            mazeList.appendChild(mazeItem);
        });
    };

    // --- Settings View ---
    const maxSpeedSlider = document.getElementById('max-speed-slider');
    const maxSpeedValue = document.getElementById('max-speed-value');
    const kpInput = document.getElementById('kp-input');
    const kiInput = document.getElementById('ki-input');
    const kdInput = document.getElementById('kd-input');

    // Mode State
    let isObstacleMode = false;

    const updateModeUI = () => {
        if (isObstacleMode) {
            loadMazeBtn.classList.add('hidden');
            solveMazeBtn.classList.add('hidden');
            obstacleModeBtn.classList.remove('hidden');
            toggleModeBtn.textContent = "Switch to Maze Solving Mode";
        } else {
            loadMazeBtn.classList.remove('hidden');
            solveMazeBtn.classList.remove('hidden');
            obstacleModeBtn.classList.add('hidden');
            toggleModeBtn.textContent = "Switch to Obstacle Mode";
        }
    };

    toggleModeBtn.addEventListener('click', () => {
        isObstacleMode = !isObstacleMode;
        updateModeUI();
        // sendMessage('abort'); // Optional: Stop current action when switching modes?
    });

    const sendSettings = () => {
        const settings = {
            max_speed: parseInt(maxSpeedSlider.value),
            kp: parseFloat(kpInput.value),
            ki: parseFloat(kiInput.value),
            kd: parseFloat(kdInput.value)
        };
        sendMessage(`settings:${JSON.stringify(settings)}`);
    };

    const updateMaxSpeedValue = () => {
        maxSpeedValue.textContent = maxSpeedSlider.value;
        sendSettings();
    };

    maxSpeedSlider.addEventListener('input', updateMaxSpeedValue);

    document.querySelectorAll('.counter-btn').forEach(btn => {
        btn.addEventListener('click', () => {
            const input = btn.parentElement.querySelector('.counter-input');
            const step = parseFloat(input.step) || 1;
            let value = parseFloat(input.value) || 0;
            if (btn.classList.contains('plus')) {
                value += step;
            } else if (btn.classList.contains('minus')) {
                value -= step;
            }
            input.value = value.toFixed(1);
            sendSettings();
        });
    });

    [kpInput, kiInput, kdInput].forEach(input => {
        input.addEventListener('change', sendSettings);
    });

    settingsBtn.addEventListener('click', () => {
        mainMenu.classList.add('hidden');
        settingsView.classList.remove('hidden');
    });

    settingsBackBtn.addEventListener('click', () => {
        mainMenu.classList.remove('hidden');
        settingsView.classList.add('hidden');
    });

    // --- Error Chart ---
    let errorChart;
    const createErrorChart = () => {
        const ctx = document.getElementById('error-chart').getContext('2d');
        errorChart = new Chart(ctx, {
            type: 'line',
            data: {
                labels: [],
                datasets: [{
                    label: 'Error',
                    data: [],
                    borderColor: getComputedStyle(document.documentElement).getPropertyValue('--celadon'),
                    backgroundColor: 'rgba(179, 222, 193, 0.1)',
                    borderWidth: 2,
                    pointRadius: 0,
                    tension: 0.4
                }]
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                scales: {
                    x: {
                        type: 'linear',
                        display: false
                    },
                    y: {
                        min: -10,
                        max: 10,
                        ticks: {
                            color: '#e0e0e0'
                        },
                        grid: {
                            color: 'rgba(224, 224, 224, 0.2)'
                        }
                    }
                },
                plugins: {
                    legend: {
                        display: false
                    }
                }
            }
        });
    };

    const addDataToChart = (data) => {
        if (!errorChart) return;
        const labels = errorChart.data.labels;
        labels.push(labels.length);
        errorChart.data.datasets[0].data.push(data);

        // Limit the number of data points
        if (labels.length > 50) {
            labels.shift();
            errorChart.data.datasets[0].data.shift();
        }
        errorChart.update();
    };

    // --- Joystick ---
    const createJoystick = () => {
        const options = {
            zone: document.getElementById('joystick-container'),
            mode: 'static',
            position: { left: '50%', top: '50%' },
            color: getComputedStyle(document.documentElement).getPropertyValue('--celadon'),
            size: 150
        };
        const manager = nipplejs.create(options);

        manager.on('move', (evt, data) => {
            const x = Math.round(Math.cos(data.angle.radian) * data.distance / (options.size / 2) * 100);
            const y = Math.round(Math.sin(data.angle.radian) * data.distance / (options.size / 2) * 100);
            sendMessage(`rc:${x},${y}`);
        });

        manager.on('end', () => {
            sendMessage('rc:0,0');
        });
    };


    // --- Event Listeners ---
    rcModeBtn.addEventListener('click', () => {
        mainMenu.classList.add('hidden');
        rcView.classList.remove('hidden');
        createJoystick();
    });

    rcBackBtn.addEventListener('click', () => {
        sendMessage('abort');
        mainMenu.classList.remove('hidden');
        rcView.classList.add('hidden');
    });

    stopBtn.addEventListener('click', () => {
        sendMessage('abort');
    });

    powerOffBtn.addEventListener('click', () => {
        powerOffModal.style.display = 'block';
    });

    confirmPowerOffBtn.addEventListener('click', () => {
        sendMessage('power_off');
        powerOffModal.style.display = 'none';
        alert('Powering off...');
    });

    cancelPowerOffBtn.addEventListener('click', () => {
        powerOffModal.style.display = 'none';
    });

    loadMazeBtn.addEventListener('click', () => {
        populateMazeList();
        loadMazeModal.style.display = 'block';
    });

    solveMazeBtn.addEventListener('click', () => {
        solveMazeModal.style.display = 'block';
    });

    obstacleModeBtn.addEventListener('click', () => {
        sendMessage('start_obstacle');
        alert('Obstacle Avoidance Started');
    });

    startSolvingBtn.addEventListener('click', () => {
        const mazeName = mazeNameInput.value.trim();
        if (mazeName) {
            sendMessage(`start_solving:${mazeName}`);
            mazeNameInput.classList.add('hidden');
            startSolvingBtn.classList.add('hidden');
            loader.classList.remove('hidden');
            abortSolvingBtn.classList.remove('hidden');
        } else {
            alert('Please enter a name for the maze.');
        }
    });

    abortSolvingBtn.addEventListener('click', () => {
        sendMessage('abort');
        resetSolveMazeModal();
    });

    loadSelectedMazeBtn.addEventListener('click', () => {
        const selectedMazeItem = document.querySelector('.maze-item.selected');
        if (selectedMazeItem) {
            const mazeName = selectedMazeItem.textContent;
            const mazes = getMazes();
            const maze = mazes.find(m => m.name === mazeName);
            if (maze) {
                sendMessage(`load_maze:${maze.solution}`);
                alert(`Sent maze "${maze.name}" to robot.`);
                loadMazeModal.style.display = 'none';
            }
        } else {
            alert('Please select a maze to load.');
        }
    });

    // --- Modal Closing Logic ---
    closeBtns.forEach(btn => {
        btn.addEventListener('click', () => {
            const modal = btn.closest('.modal');
            if (modal) {
                modal.style.display = 'none';
            }
        });
    });

    window.addEventListener('click', (event) => {
        if (event.target.classList.contains('modal')) {
            event.target.style.display = 'none';
        }
    });

    createErrorChart();
});
