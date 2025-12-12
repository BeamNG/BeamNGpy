let selectedVehicleIndex = 0
let vehicleList = []
let initialized = false
let schemas = null
let currentVehicle = null
let currentPage = null
let settings = null

async function init() {
    await loadSchemas()
    await updateVehicleList()

    setInterval(updateVehicleList, 2000)
    document.getElementById("delete-vehicle").addEventListener("click", openDeleteVehicleModal)
    document.getElementById("confirm-delete-vehicle").addEventListener("click", confirmDeleteVehicle)
    document.getElementById("open-vehicle-file").addEventListener("click", openVehicleFile)
    document.getElementById("reload-vehicle-file").addEventListener("click", reloadVehicleFile)
    document.getElementById("generate-vehicle").addEventListener("click", generateVehicle)
    document.getElementById("play-vehicle").addEventListener("click", playVehicle)
    document.getElementById("create-vehicle").addEventListener("click", openCreateVehicleModal)
    document.getElementById("create-vehicle-action").addEventListener("click", openCreateVehicleModal)
    document.getElementById("confirm-create-vehicle").addEventListener("click", confirmCreateVehicle)
    document.getElementById("open-settings").addEventListener("click", openSettings)
    document.getElementById("open-settings-file").addEventListener("click", openSettingsFile)
    document.getElementById("open-vehicles-folder").addEventListener("click", openVehiclesFolder)
    document.getElementById("open-mods-folder").addEventListener("click", openModsFolder)
    resetProgressAreaText()
    initialized = true
}

async function loadSchemas() {
    try {
        const response = await fetch("/schemas")
        await checkResponseOk(response, "Failed to fetch schemas")
        schemas = await response.json()
        const keys = Object.keys(schemas)
        for (const key of keys) {
            if (schemas[key]['$defs']) {
                for (const k in schemas[key]['$defs']) {
                    value = schemas[key]['$defs'][k]
                    schemas[k] = value
                }
            }
        }
    } catch (error) {
        showError("Loading data schemas failed: " + error.message)
    }
}

async function updateVehicleList() {
    try {
        const response = await fetch("/vehicles")
        await checkResponseOk(response, "Failed to fetch vehicle list")
        const data = await response.json()
        const list = document.getElementById("vehicle-list")
        if (initialized && data.length == vehicleList.length) {
            // check if all elements are equal
            if (data.every((value, index) => (
                value.id == vehicleList[index].id &&
                value.name == vehicleList[index].name &&
                value.is_generated == vehicleList[index].is_generated
            ))) {
                return
            }
        }
        list.innerHTML = ""
        vehicleList = data
        for (const [ index, vehicle ] of data.entries()) {
            const button = document.createElement("button")
            button.classList.add("list-group-item", "list-group-item-action")
            button.textContent = getDisplayName(vehicle)
            button.addEventListener("click", () => selectVehicle(index))
            list.appendChild(button)
        }
        await selectVehicle(selectedVehicleIndex)
    } catch (error) {
        showError("Updating vehicle list failed: " + error.message)
    }
}

function getDisplayName(vehicle) {
    return vehicle.name || vehicle.id
}

async function selectVehicle(index) {
    if (index >= vehicleList.length) {
        index = 0
    }
    if (index != selectedVehicleIndex) {
        resetProgressAreaText()
    }
    selectedVehicleIndex = index
    const list = document.getElementById("vehicle-list")
    for (const [ index, button ] of Array.from(list.children).entries()) {
        button.classList.remove("active")
        if (index == selectedVehicleIndex) {
            button.classList.add("active")
        }
    }
    const page = (selectedVehicleIndex >= vehicleList.length) ? "no-vehicles" : "vehicle"
    await updatePage(page)
}

function selectNoVehicle() {
    const list = document.getElementById("vehicle-list")
    for (const button of list.children) {
        button.classList.remove("active")
    }
}

async function updatePage(newPage) {
    let differentPage = newPage != currentPage
    currentPage = newPage
    const pages = {
        "vehicle": updateVehiclePage,
        "no-vehicles": updateNoVehiclesPage,
        "settings": updateSettingsPage,
    }
    if (differentPage) {
        for (const page in pages) {
            const pageElement = document.getElementById(page + "-page")
            pageElement.classList.add("d-none")
        }
    }
    const func = pages[currentPage]
    if (func) {
        await func()
    }
    if (differentPage) {
        document.getElementById(currentPage + "-page").classList.remove("d-none")
    }
}

async function updateVehiclePage() {
    const vehicleId = vehicleList[selectedVehicleIndex].id
    const id = document.getElementById("vehicle-id")
    id.textContent = "ID: " + vehicleId
    await loadVehicleData(vehicleId)
}

async function updateNoVehiclesPage() {
    currentVehicle = null
}

async function loadVehicleData(vehicleId) {
    try {
        const response = await fetch(`/vehicles/${vehicleId}`)
        await checkResponseOk(response, "Failed to fetch vehicle data")
        const data = await response.json()
        currentVehicle = data
        updateVehicleForm()
    } catch (error) {
        showError("Loading vehicle data failed: " + error.message)
    }
}

function updateVehicleForm() {
    const form = document.getElementById("vehicle-form")
    const vehicleData = currentVehicle
    const numberOfVariables = Object.values(currentVehicle.optimization.enabled).filter(value => value === true).length
    const needsOptimization = numberOfVariables > 0
    const playButton = document.getElementById("play-vehicle")
    playButton.disabled = !vehicleList[selectedVehicleIndex].is_generated
    form.innerHTML = ""
    generateForm(form, vehicleData, "TemplateVehicle", "form-vehicle", saveVehicleData)
    setProgressAreaVisible(needsOptimization)
    const generateButtonText = document.getElementById("generate-vehicle-text")
    generateButtonText.textContent = (needsOptimization) ? "Optimize" : "Generate"
}

function generateForm(form, data, schemaName, id, saveCallback) {
    const schema = schemas[schemaName]
    for (const propertyName in schema.properties) {
        const def = schema.properties[propertyName]
        const value = data[propertyName]
        const title = def.title
        const description = def.description
        let type = ""
        let propSchemaName = null
        let subSchema = null
        const fieldId = id + "-" + propertyName
        if (def['$ref']) {
            propSchemaName = def['$ref'].split("/").pop()
            type = schemas[propSchemaName].type
            subSchema = schemas[propSchemaName]
        } else {
            type = def.type
        }
        if (type == "object") {
            const section = document.createElement("h5")
            section.textContent = title
            form.appendChild(section)
            const row = document.createElement("div")
            row.classList.add("row", "g-3", "mb-3")
            if (propertyName == "optimization") {
                generateOptimizationForm(row, value, propSchemaName, fieldId, saveCallback)
            } else {
                generateForm(row, value, propSchemaName, fieldId, saveCallback)
            }
            form.appendChild(row)
        } else {
            if (type == "string") {
                if (subSchema && Array.isArray(subSchema.enum)) {
                    let el = generateSelectInput(form, fieldId, value, title, description, subSchema.enum)
                    addStringListener(el, data, propertyName, saveCallback)
                } else {
                    let el = generateStringInput(form, fieldId, value, title, description)
                    addStringListener(el, data, propertyName, saveCallback)
                }
            } else if (type == "number") {
                let el = generateNumberInput(form, fieldId, value, title, description, def.minimum, def.maximum)
                addNumberListener(el, data, propertyName, saveCallback, def)
            } else {
                throw new Error("Unknown form data type: " + type)
            }
        }
    }
}

function generateOptimizationForm(form, data, schemaName, id, saveCallback) {
    const schema = schemas[schemaName]
    const targetSchemaName = schema.properties.targets.$ref.split("/").pop()
    const targetsSchema = schemas[targetSchemaName]
    const variableSchemaName = schema.properties.variables.$ref.split("/").pop()
    const variablesSchema = schemas[variableSchemaName]
    for (const propertyName in targetsSchema.properties) {
        const varName = propertyName + "_factor"
        const defTarget = targetsSchema.properties[propertyName]
        const defVar = variablesSchema.properties[varName]
        const enabled = data.enabled[propertyName] || false
        const title = defTarget.title
        let value, def, fieldId, propName, propData, helpText, btnText
        if (enabled) {
            value = data.targets[propertyName]
            def = defTarget
            fieldId = id + "-targets-" + propertyName
            propName = propertyName
            propData = data.targets
            helpText = "Scaling factor is optimized to meet target."
            btnText = "Target"
        } else {
            value = data.variables[varName]
            def = defVar
            fieldId = id + "-variables-" + varName
            propName = varName
            propData = data.variables
            helpText = "Scaling factor is directly set by user."
            btnText = "Factor"
        }
        let description = def.description
        if (description == "") {
            description = helpText
        } else {
            description = description + ", " + helpText
        }
        const type = def.type
        if (type == "number") {
            const btn = document.createElement("button")
            btn.className = "btn btn-outline-secondary"
            btn.type = "button"
            btn.textContent = btnText
            btn.addEventListener("click", async () => {
                data.enabled[propertyName] = !enabled
                await saveCallback()
                await reloadVehicleFile()
            })
            let el = generateNumberInput(form, fieldId, value, title, description, def.minimum, def.maximum, btn)
            addNumberListener(el, propData, propName, saveCallback, def)
        } else {
            throw new Error("Unknown optimization data type: " + type)
        }
    }
}

function generateStringInput(form, id, value, title, description) {
    const div = document.createElement("div")
    div.classList.add("col-12", "mb-3")
    const label = document.createElement("label")
    label.classList.add("form-label")
    label.textContent = title
    label.setAttribute("for", id)
    div.appendChild(label)
    const input = document.createElement("input")
    input.classList.add("form-control")
    input.setAttribute("id", id)
    input.value = value
    div.appendChild(input)
    if (description) {
        const text = document.createElement("div")
        text.classList.add("form-text")
        text.textContent = description
        div.appendChild(text)
    }
    form.appendChild(div)
    return input
}

function generateSelectInput(form, id, value, title, description, options) {
    const div = document.createElement("div")
    div.classList.add("col-12", "col-md-6", "col-lg-4")
    const label = document.createElement("label")
    label.classList.add("form-label")
    label.textContent = title
    label.setAttribute("for", id)
    div.appendChild(label)
    const select = document.createElement("select")
    select.classList.add("form-select")
    select.setAttribute("id", id)
    for (const option of options) {
        const opt = document.createElement("option")
        opt.value = option
        opt.textContent = option
        select.appendChild(opt)
    }
    div.appendChild(select)
    if (description) {
        const text = document.createElement("div")
        text.classList.add("form-text")
        text.textContent = description
        div.appendChild(text)
    }
    select.value = value
    form.appendChild(div)
    return select
}

function generateNumberInput(form, id, value, title, description, minimum, maximum, button) {
    const div = document.createElement("div")
    div.classList.add("col-12", "col-md-6", "col-lg-4")
    const label = document.createElement("label")
    label.classList.add("form-label")
    label.textContent = title
    label.setAttribute("for", id)
    div.appendChild(label)
    const group = document.createElement("div")
    group.classList.add("input-group")
    const input = document.createElement("input")
    input.setAttribute("type", "number")
    input.classList.add("form-control")
    input.setAttribute("id", id)
    input.setAttribute("step", "0.01")
    input.value = value
    if (button) group.appendChild(button)
    group.appendChild(input)
    const unitInfo = stripUnit(description)
    description = unitInfo.description
    const unit = unitInfo.unit
    if (!description) description = ""
    if (minimum) input.setAttribute("min", "" + minimum)
    if (maximum) input.setAttribute("max", "" + maximum)
    if (unit) {
        const unitText = document.createElement("span")
        unitText.classList.add("input-group-text")
        unitText.textContent = unit
        group.appendChild(unitText)
    }
    div.appendChild(group)
    if (description) {
        const text = document.createElement("div")
        text.classList.add("form-text")
        text.textContent = description
        div.appendChild(text)
    }
    form.appendChild(div)
    return input
}

function setProgressAreaVisible(visible) {
    const progressArea = document.getElementById("progress-area")
    if (visible) {
        progressArea.classList.remove("d-none")
    } else {
        progressArea.classList.add("d-none")
    }
}

function isProgressAreaVisible() {
    const progressArea = document.getElementById("progress-area")
    return !progressArea.classList.contains("d-none")
}

function resetProgressAreaText() {
    const progressText = document.getElementById("progress-text")
    const lines = [
        "Optimization required.",
        "BeamNG will be used during the optimization process.",
        "Status will be shown here.",
    ]
    progressText.textContent = lines.join("\n")
}

function stripUnit(description) {
    if (!description) {
        return { unit: "", description: "" }
    }
    let unit = ""
    const parts = description.split(", ")
    const newParts = []
    for (const part of parts) {
        if (part.startsWith("unit: ")) {
            unit = part.split(": ")[1].trim()
        } else {
            newParts.push(part)
        }
    }
    return { unit: unit, description: newParts.join(", ") }
}

function addStringListener(el, data, propertyName, callback) {
    el.addEventListener("change", async (event) => {
        data[propertyName] = event.target.value
        await callback()
    })
}

function addNumberListener(el, data, propertyName, callback, def) {
    el.addEventListener("change", async (event) => {
        let num = parseFloat(event.target.value)
        if (isNaN(num)) num = def.default
        if (num < def.minimum) num = def.minimum
        if (num > def.maximum) num = def.maximum
        data[propertyName] = num
        el.value = num
        await callback()
    })
}

async function checkResponseOk(response, message) {
    if (!message) {
        message = "Error"
    }
    if (!response.ok) {
        let detail = null
        try {
            const data = await response.json()
            detail = data.detail
        } catch {
            detail = null
        }
        if (detail) {
            if (Array.isArray(detail)) {
                detail = parseDetailMessage(detail)
            }
            message = message + ` (status ${response.status}): ` + detail
        }
        throw new Error(message)
    }
}

function parseDetailMessage(detail) {
    let html = "<ul class='mb-0'>"
    for (const item of detail) {
        const locStr = item.loc.join(".")
        html += `<li>${locStr}: ${item.msg}</li>`
    }
    html += "</ul>"
    return html
}

function showError(message) {
    const errorPlaceholder = document.getElementById("error-placeholder")
    const icon = `<i class="bi bi-exclamation-triangle-fill me-2"></i>`
    errorPlaceholder.innerHTML = `<div class="alert alert-danger">${icon}${message}</div>`
    scrollTo(0, 0)
}

function hideError() {
    const errorPlaceholder = document.getElementById("error-placeholder")
    errorPlaceholder.innerHTML = ""
}

async function writeJson(method, url, data) {
    return await fetch(url, {
        method: method,
        headers: {
            "Content-Type": "application/json"
        },
        body: JSON.stringify(data)
    })
}

async function saveVehicleData() {
    try {
        let data = currentVehicle
        let id = vehicleList[selectedVehicleIndex].id
        const response = await writeJson("PUT", `/vehicles/${id}`, data)
        await checkResponseOk(response, "Failed to save vehicle data")
        if (currentVehicle.name != vehicleList[selectedVehicleIndex].name) {
            updateVehicleNameInList()
        }
    } catch (error) {
        showError("Saving vehicle data failed: " + error.message)
    }
}

function updateVehicleNameInList() {
    vehicleList[selectedVehicleIndex].name = currentVehicle.name
    const list = document.getElementById("vehicle-list")
    for (const [ index, button ] of Array.from(list.children).entries()) {
        if (index == selectedVehicleIndex) {
            button.textContent = getDisplayName(vehicleList[selectedVehicleIndex])
        }
    }
}

function openDeleteVehicleModal() {
    const modal = new bootstrap.Modal(document.getElementById('delete-vehicle-modal'))
    modal.show()
}

function closeDeleteVehicleModal() {
    const modal = bootstrap.Modal.getInstance(document.getElementById('delete-vehicle-modal'))
    modal.hide()
}

async function confirmDeleteVehicle() {
    try {
        closeDeleteVehicleModal()
        const id = vehicleList[selectedVehicleIndex].id
        const response = await fetch(`/vehicles/${id}`, {
            method: "DELETE"
        })
        await checkResponseOk(response, "Failed to delete vehicle")
        await updateVehicleList()
    } catch (error) {
        showError("Deleting vehicle failed: " + error.message)
    }
}

async function openVehicleFile() {
    try {
        const id = vehicleList[selectedVehicleIndex].id
        const response = await fetch(`/ui/open/vehicles/${id}`, {method: "POST"})
        await checkResponseOk(response, "Failed to open vehicle file")
    } catch (error) {
        showError("Opening vehicle file failed: " + error.message)
    }
}

async function reloadVehicleFile() {
    await loadVehicleData(vehicleList[selectedVehicleIndex].id)
}

async function generateVehicle() {
    hideError()
    const generateButton = document.getElementById("generate-vehicle")
    const playButton = document.getElementById("play-vehicle")
    const generatingStatus = document.getElementById("generating-vehicle")
    const generatedStatus = document.getElementById("generated-vehicle")
    const failedStatus = document.getElementById("generate-vehicle-failed")
    
    // Hide all status elements and disable button
    generatingStatus.classList.add("d-none")
    generatedStatus.classList.add("d-none")
    failedStatus.classList.add("d-none")
    generateButton.disabled = true
    generateButton.style.cursor = "not-allowed"
    playButton.disabled = true
    playButton.style.cursor = "not-allowed"

    // Set a timeout to show generating status after 500ms
    const generatingTimeout = setTimeout(() => {
        generatingStatus.classList.remove("d-none")
    }, 500)
    
    let success = true
    try {
        const id = vehicleList[selectedVehicleIndex].id
        const response = await fetch(`/vehicles/${id}/generate`, {
            method: "POST"
        })
        await showProgress(response)

        // Show success status
        clearTimeout(generatingTimeout)
        generatingStatus.classList.add("d-none")
        generatedStatus.classList.remove("d-none")
        setTimeout(() => {
            generatedStatus.classList.add("d-none")
        }, 5000)
    } catch (error) {
        // Show failure status
        clearTimeout(generatingTimeout)
        generatingStatus.classList.add("d-none")
        failedStatus.classList.remove("d-none")
        setTimeout(() => {
            failedStatus.classList.add("d-none")
        }, 5000)
        if (!error.message.includes("Cancelled by user.")) {
            showError("Generating vehicle failed: " + error.message)
        }
        success = false
    } finally {
        generateButton.disabled = false
        generateButton.style.cursor = "pointer"
        playButton.disabled = false
        playButton.style.cursor = "pointer"
    }
    return success
}

async function cancelGeneration(id) {
    try {
        const response = await fetch(`/vehicles/${id}/cancel`, {
            method: "POST"
        })
        await checkResponseOk(response, "Failed to cancel generation")
    } catch (error) {
        showError("Cancelling generation failed: " + error.message)
    }
}

async function showProgress(response) {
    let progress = null
    const allLines = []
    if (isProgressAreaVisible()) {
        progress = document.getElementById("progress-text")
        progress.textContent = ""
    }

    function handleLine(line) {
        allLines.push(line)
        if (progress) {
            progress.textContent += line + "\n"
            progress.scrollTo({ top: progress.scrollHeight, behavior: "smooth" })
        }
    }

    if (!response.ok) {
        const text = await response.text()
        throw new Error(text + ` (status ${response.status})`)
    }
    const reader = response.body
        .pipeThrough(new TextDecoderStream()) // decode UTF-8 to text
        .getReader()
    await readLines(reader, handleLine)

    if (allLines.includes("--- ERROR ---")) {
        const errorIndex = allLines.indexOf("--- ERROR ---")
        const errorLines = allLines.slice(errorIndex + 1)
        const errorMessage = errorLines.join("\n")
        throw new Error(errorMessage)
    }
}

async function readLines(reader, callback) {
    // read lines from reader and call callback for each line
    let buffer = ""

    while (true) {
        const { value, done } = await reader.read()
        if (done) break

        buffer += value

        // split by newlines into complete lines
        const lines = buffer.split("\n")

        // keep the last partial line in the buffer
        buffer = lines.pop();

        for (const line of lines) {
            callback(line)
        }
    }

    // emit any final line when the stream closes
    if (buffer.length > 0) {
        callback(buffer)
    }
}

async function playVehicle() {
    const vehicleId = vehicleList[selectedVehicleIndex].id
    const levelId = "smallgrid"
    success = await runBeamNG(vehicleId, levelId)
    return success
}

async function runBeamNG(vehicleId, levelId) {
    try {
        const response = await fetch(`/vehicles/${vehicleId}/play/${levelId}`, {
            method: "POST"
        })
        await checkResponseOk(response, "Failed to launch BeamNG")
    } catch (error) {
        showError("Launching BeamNG failed: " + error.message)
        success = false
    }
    return success
}

function idFromName(input) {
    // Make string lowercase
    let result = input.toLowerCase()
    
    // Replace all non-[a-z0-9_] characters with underscore
    result = result.replace(/[^a-z0-9_]/g, '_')
    
    // Combine consecutive underscores to one underscore
    result = result.replace(/_+/g, '_')
    
    // Strip underscores from beginning and end
    result = result.replace(/^_+|_+$/g, '')

    // If longer than 50 characters, truncate
    if (result.length > 50) {
        result = result.slice(0, 50)
    }
    
    // If empty, make it "template_car_1"
    if (result === '') {
        result = 'template_car_1'
    }
    
    // If doesn't start with [a-z], add "template_car_" at the beginning
    if (!/^[a-z]/.test(result)) {
        result = 'template_car_' + result
    }
    
    return result
}

function toUniqueId(candidateId, existingIds) {
    let uniqueId = candidateId
    while (existingIds.includes(uniqueId)) {
        // Check if the ID ends with a digit
        const digitMatch = uniqueId.match(/(\d+)$/)
        if (digitMatch) {
            // Extract the number part and increment it
            let numberPart = parseInt(digitMatch[1])
            if (!Number.isSafeInteger(numberPart)) {  // edge case where number == number + 1
                numberPart = 0  // assuming the user does not have so many existing ids
            }
            const basePart = uniqueId.slice(0, -digitMatch[1].length)
            uniqueId = basePart + (numberPart + 1)
        } else {
            // Doesn't end with digit
            if (uniqueId.endsWith('_')) {
                uniqueId = uniqueId + '1'
            } else {
                uniqueId = uniqueId + '_1'
            }
        }
    }
    return uniqueId
}

function openCreateVehicleModal() {
    const modalElement = document.getElementById('create-vehicle-modal')
    const modal = new bootstrap.Modal(modalElement)
    document.getElementById("new-vehicle-name").value = ""
    // Focus after modal is shown
    modalElement.addEventListener('shown.bs.modal', function () {
        document.getElementById("new-vehicle-name").focus()
    }, { once: true })
    modal.show()
}

function closeCreateVehicleModal() {
    const modal = bootstrap.Modal.getInstance(document.getElementById('create-vehicle-modal'))
    modal.hide()
}

async function confirmCreateVehicle() {
    closeCreateVehicleModal()
    const name = document.getElementById("new-vehicle-name").value
    await createVehicle(name)
}

async function createVehicle(name) {
    try {
        const id = idFromName(name)
        const uniqueId = toUniqueId(id, vehicleList.map(v => v.id))
        const data = { name: name }
        const response = await writeJson("POST", `/vehicles/${uniqueId}`, data)
        await checkResponseOk(response, "Failed to create vehicle")
        await updateVehicleList()
        let index = vehicleList.findIndex(v => v.id == uniqueId)
        await selectVehicle(index)
    } catch (error) {
        showError("Creating vehicle failed: " + error.message)
    }
}

function openSettings() {
    selectNoVehicle()
    updatePage("settings")
}

async function updateSettingsPage() {
    try {
        const response = await fetch(`/settings`)
        await checkResponseOk(response, "Failed to fetch settings")
        const data = await response.json()
        settings = data
        updateSettingsForm()
    } catch (error) {
        showError("Loading settings failed: " + error.message)
    }
}

function updateSettingsForm() {
    const form = document.getElementById("settings-form")
    const settingsData = settings
    form.innerHTML = ""
    generateForm(form, settingsData, "Settings", "form-settings", saveSettingsData)
}

async function saveSettingsData() {
    try {
        let data = settings
        const response = await writeJson("PUT", `/settings`, data)
        await checkResponseOk(response, "Failed to save settings")
    } catch (error) {
        showError("Saving settings failed: " + error.message)
    }
}

async function openSettingsFile() {
    try {
        const response = await fetch(`/ui/open/settings`, {method: "POST"})
        await checkResponseOk(response, "Failed to open settings file")
    } catch (error) {
        showError("Opening settings file failed: " + error.message)
    }
}

async function openVehiclesFolder() {
    try {
        const response = await fetch(`/ui/open/vehicles`, {method: "POST"})
        await checkResponseOk(response, "Failed to open vehicles folder")
    } catch (error) {
        showError("Opening vehicles folder failed: " + error.message)
    }
}

async function openModsFolder() {
    try {
        const response = await fetch(`/ui/open/mods`, {method: "POST"})
        await checkResponseOk(response, "Failed to open mods folder")
    } catch (error) {
        showError("Opening mods folder failed: " + error.message)
    }
}

document.addEventListener("DOMContentLoaded", init)
