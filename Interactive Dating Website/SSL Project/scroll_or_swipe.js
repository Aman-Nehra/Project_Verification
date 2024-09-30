document.addEventListener("DOMContentLoaded", function () {//when DOM is fully loaded this function is called
    const dropdown = document.getElementById('dropdown');//get dropdown element
    dropdown.addEventListener('change', filterProfiles); // Add event listener to the dropdown
    loadProfiles(); // Load all profiles initially
});
//function to load profiles from JSON
function loadProfiles() {
    fetch('students.json') //fetch JSON data
        .then(response => response.json()) 
        .then(data => {
            const profilesContainer = document.getElementById('profilesContainer'); //Get profiles container
            data.forEach(student => {
                const profileCard = createProfileCard(student); // Create profile card for each student
                profilesContainer.appendChild(profileCard); // Append profile card to container
            });
        })
        .catch(error => console.error('Error loading profiles:', error));
}

function filterProfiles() {
    fetch('students.json')
        .then(response => response.json())
        .then(data => {
            const profilesContainer = document.getElementById('profilesContainer');
            const selectedGender = dropdown.value.toLowerCase(); // Get the selected gender from the dropdown

            // Clear previous profile cards
            profilesContainer.innerHTML = '';

            // Filter data based on selected gender
            const filteredData = data.filter(student => student["Gender"].toLowerCase() === selectedGender );

            // Create profile cards for filtered data
            filteredData.forEach(student => {
                const profileCard = createProfileCard(student); //create profile card for each student
                profilesContainer.appendChild(profileCard); //Append profile card to container
            });
        })
        .catch(error => console.error('Error filtering profiles:', error));
}

//function to create profile card HTML
function createProfileCard(student) {
    const profileCard = document.createElement('div'); //Create profile card element
    profileCard.classList.add('profile'); // Add "profile" class to profile card
    // provide student details in the profile card as innerHTML
    profileCard.innerHTML = `
        <img src="${student["Photo"]}" alt="Profile Photo" class="photo" style="height:200px">
        <h2>${student["Name"]}</h2>
        <p>Roll Number: ${student["IITB Roll Number"]}</p>
        <p>Year of Study: ${student["Year of Study"]}</p>
        <p>Age: ${student["Age"]}</p>
        <p>Gender: ${student["Gender"]}</p>
        <p>Interests: ${student["Interests"].join(", ")}</p>
        <p>Hobbies: ${student["Hobbies"].join(", ")}</p>
        <p>Email: ${student["Email"]}</p>
    `;
    // Retrieve the number of likes from localStorage
    let likes = localStorage.getItem(student["Name"]) || 0;
    const like_count = document.createElement('span'); // Create element to display like count
    like_count.textContent = likes + ' Likes';//set initial like count text
    like_count.style.marginLeft = '10px';
    const unlike_image = document.createElement('img')//create image element for like button
    unlike_image.src='Images/Screenshot 2024-04-24 213828.png';
    unlike_image.alt = "unlike";
    unlike_image.style.height="50px";
    let isLiked = false; //variable declared to  track like status
    unlike_image.addEventListener('click', function () {
        if (isLiked) {
            // If already liked, revert to the original image
            unlike_image.src = 'Images/Screenshot 2024-04-24 213828.png'; // Set the initial image source
            likes--;//decrement of like count
            like_count.textContent=likes + ' Likes';//Updat elike counts
        } else {
            // If not liked, change to some other image
            unlike_image.src = 'Images/Screenshot 2024-04-24 213842.png'; // Set the new image source
            likes++; //increment like count
            like_count.textContent=likes + ' Likes'; //Update like count text
        }
        // Toggle like state
        isLiked = !isLiked;
        // Store the updated like count in localStorage
        localStorage.setItem(student["Name"], likes);
    });
    profileCard.appendChild(unlike_image); //Append the unlike image to the profile card
    profileCard.appendChild(like_count); //Append the like count to the profile card
    return profileCard;
}

