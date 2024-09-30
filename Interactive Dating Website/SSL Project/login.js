function check(data){
    let present = false;
for(x of data){
    if(document.getElementById("username").value==x["username"]){
        present = true;   //if correct username is provided then present become true
        break;
    }
}
if(present==false){
    document.getElementById("error").innerHTML="ERROR:Invalid Username"; //if after iterating over complete data the value of present is false then a message  of invalid username will be given.
}
for(x of data){
if(document.getElementById("username").value==x["username"]){
    if(document.getElementById("password").value!=x["password"]){   
        document.getElementById("error").innerHTML="Error: Incorrect Password";  //if the username is present in the data but the password provided corresponding to it is incorrect then a message of invalid password will be given.
    }
}
}
for(x of data){
    if(document.getElementById("username").value==x["username"]){
        if(document.getElementById("password").value==x["password"]){
            document.getElementById("login").innerHTML = '<a href="dating.html">LOGIN</a>';
        }    //if the password provided corresponding to username is correct then on clicking login button dating.html page will open.
    }
}
}


// the login.json file is linked to login.html and is given an id of "json-data".
var scriptElement = document.getElementById("json-data");

// Getting the source URL of the JSON file from the script tag
var jsonUrl = scriptElement.src;
// Making a fetch request to get the JSON data
fetch(jsonUrl)
  .then(response => response.json())
  .then(data => {
    document.getElementById("login").addEventListener("click",()=>{ check(data) })//onclicking the login button check(data) function is called.
  })

  
