document.addEventListener("DOMContentLoaded", function () {
    // Declaring variables for later use
    let data;
    let selectedGender;
    const selectedInterests = [];
    let perfect;
    // Fetching JSON data
    fetchJsonData();

    // Function to fetch JSON data
    function fetchJsonData() {
        var scriptElement = document.getElementById("json-data");
        var jsonUrl = scriptElement.src;

        fetch(jsonUrl)
            .then(response => response.json())
            .then(jsonData => {
                data = jsonData; // Storing data in a global variable to use it outside the scope
            })
            .catch(error => {
                console.error('Error fetching JSON data:', error);
            });
    }

    // Function to handle checkbox change event i.e. when  every time the checkbox changes the selectedInterests array becomes empty and it is again filled with all the checked items 
    function handleCheckboxChange() {
        selectedInterests.length = 0; // Clear the array
        document.querySelectorAll('input[name=check].input').forEach(checkbox => {
            if (checkbox.checked) {
                selectedInterests.push(checkbox.value); //if the interest is checked then it is added in the array
            }
        });
    }

    // Function to handle radio button change event i.e. when every the gender radio button is changed the selectedGender is also  changed.
    function handleRadioChange() {
        selectedGender = document.querySelector('input[name=gender]:checked').value;
    }

    // Function to handle Find a Match button click event.
    function handleFindMatchClick() {
        perfectMatch();
    }

    // Adding event listeners to checkboxes, radio buttons, and Find a Match button
    document.querySelectorAll('input[name=check].input').forEach(checkbox => {
        checkbox.addEventListener('change', handleCheckboxChange);//adding the change event to handleCheckboxChange function
    });
    document.querySelectorAll('input[name=gender]').forEach(radio => {
        radio.addEventListener('change', handleRadioChange);//adding the change event to handleRadioChange function
    });
    document.getElementById('Submit').addEventListener('click', handleFindMatchClick);//onclicking the find a match button handleFindMatchClick function is called which in turn calls the perfectMatch function.Directly calling the perfectMatch function instead of wrapping it inside handleFindMatchClick could lead to the perfectMatch function being executed before the necessary data is ready or before the user has made their selections.

    // Function to find the perfect match based on shared interests
    function perfectMatch() {
        let counterArray = []; 
        let genderFilteredArray = [];
        for (let person of data) {
            if (person["Gender"] === selectedGender) {
                continue; // Skip same gender
            }
            genderFilteredArray.push(person); //genderFilteredArray contains the data of only opposite gender
            let counter = 0; //initialize counter to zero 
            for (let interest of person["Interests"]) {
                if (selectedInterests.includes(interest)) {
                    counter++;//if the interest of the profile  is included in the selectedInterests then the counter increases by one
                }
            }
            counterArray.push(counter);//add the counter to the counter array
        }
        // Check if all elements of counter_array are zero 
        const allZero = counterArray.every(count => count === 0);//if all elements are zero then no match is suitable
        if (allZero) {
            document.getElementById("match").innerHTML = '<p style="font-size:20px">Sorry, we are unable to find a perfect match for you</p>' + '<img id="sorry" src="Images/Screenshot 2024-04-24 233028.png" alt="image" width="100px">';
            return; // Exit the function if no match is found
        }
        // Find the person with the maximum number of shared interests
        let maxIndex = 0;
        for (let i = 0; i < counterArray.length; i++) {
            if (counterArray[maxIndex] < counterArray[i]) {
                maxIndex = i;
            }
        }
        perfect = genderFilteredArray[maxIndex];//perfect contains the details of the perfect match
        displayMatch(perfect);
    }

    // Function to display the matched person's information
    function displayMatch(perfect) {
        let div1 = document.createElement("div"); //a div for showing the perfect match is created 
        div1.setAttribute("id", "profile"); // the id of the div is set  to  profile
        document.body.appendChild(div1); // the div is appended to the body
        let div_1 = document.getElementById("profile");
        div_1.innerHTML = '<img src="' + perfect["Photo"] + '" id="photo_match">' +  //filling the innerHTML of the div with various tags
            '<p id="IITB_Roll_Number_match"></p>' +
            '<p id="name_match"></p>' +
            '<p id="Year_of_Study_match"></p>' +
            '<p id="Age_match"></p>' +
            '<p id="Gender_match"></p>' +
            '<p id="Interests_match"></p>' +
            '<p id="Hobbies_match"></p>' +
            '<p id="Email_match"></p>' +
            '<button id="send" onmouseover="mOver(this)" onmouseout="mOut(this)"><a href="mailto:' + perfect["Email"] + '?subject=You Got a Perfect Match!!&body=Hi ' + perfect["Name"] + ',%0D%0A%0D%0ACongratulations! You\'ve found a perfect match on Love Link.%0D%0A%0D%0AHere are the details:%0D%0A%0D%0ARoll Number: ' + document.getElementById("rollNumber").value + '%0D%0AName: ' + document.getElementById("name").value + '%0D%0AYear of Study: ' + document.getElementById("year").value + '%0D%0AAge: ' + document.getElementById("age").value + '%0D%0AGender: ' + document.getElementById("gender").value + '%0D%0A%0D%0AEmail: ' + document.getElementById("email").value + '%0D%0A%0D%0AFeel free to reach out and start a conversation!%0D%0A%0D%0ABest regards,%0D%0ALove Link">Send Email</a></button>'//%0A%0D is used for new line in URL endcoding
            //button to send personalized email to the perfect match
            //filling the innerHTML of the div with the details of the perfect match
        document.getElementById("match").innerHTML = '<i>PERFECT MATCH</i>';
        document.getElementById("IITB_Roll_Number_match").innerHTML = "Roll Number: " + perfect["IITB Roll Number"];
        document.getElementById("name_match").innerHTML = "Name: " + perfect["Name"];
        document.getElementById("Year_of_Study_match").innerHTML = "Year of Study: " + perfect["Year of Study"];
        document.getElementById("Age_match").innerHTML = "Age: " + perfect["Age"];
        document.getElementById("Gender_match").innerHTML = "Gender: " + perfect["Gender"];
        document.getElementById("Interests_match").innerHTML = "Interests: " + perfect["Interests"].join(", ");
        document.getElementById("Hobbies_match").innerHTML = "Hobbies: " + perfect["Hobbies"].join(", ");
        document.getElementById("Email_match").innerHTML = "Email: " + perfect["Email"];

       
    }
});

