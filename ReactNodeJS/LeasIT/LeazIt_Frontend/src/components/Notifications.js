import React, { Component } from 'react';
import { ToastContainer, toast } from 'react-toastify';
import 'react-toastify/dist/ReactToastify.css';
import { withRouter } from 'react-router-dom'

// minified version is also included
import 'react-toastify/dist/ReactToastify.min.css';

class Example extends Component {
    notify(){
        // toast("Default Notification !");

        toast.success("Success", {
            position: toast.POSITION.TOP_CENTER
        });
        //
        // toast.error("Error Notification !", {
        //     position: toast.POSITION.TOP_LEFT
        // });
        //
        // toast.warn("Warning Notification !", {
        //     position: toast.POSITION.BOTTOM_LEFT
        // });
        //
        // toast.info("Info Notification !", {
        //     position: toast.POSITION.BOTTOM_CENTER
        // });
        //
        // toast("Custom Style Notification with css class!", {
        //     position: toast.POSITION.BOTTOM_RIGHT,
        //     className: 'foo-bar'
        // });
    };

    render(){
        return (
            <div>
                <button onClick={this.notify}>Notify</button>
                <ToastContainer />
            </div>
        );
    }
}

// class Example extends Component {
//     notify(){
//         toast("Default Notification !");
//
//         toast.success("Success Notification !", {
//             position: toast.POSITION.TOP_CENTER
//         });
//
//         toast.error("Error Notification !", {
//             position: toast.POSITION.TOP_LEFT
//         });
//
//         toast.warn("Warning Notification !", {
//             position: toast.POSITION.BOTTOM_LEFT
//         });
//
//         toast.info("Info Notification !", {
//             position: toast.POSITION.BOTTOM_CENTER
//         });
//
//         toast("Custom Style Notification with css class!", {
//             position: toast.POSITION.BOTTOM_RIGHT,
//             className: 'foo-bar'
//         });
//     };
//
//     render(){
//         return <button onClick={this.notify}>Notify</button>;
//     }
// }

export default withRouter(Example)