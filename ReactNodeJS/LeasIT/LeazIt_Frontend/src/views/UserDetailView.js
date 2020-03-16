"use strict";

import React from 'react';
import UserDetail  from '../components/UserDetail';
import UserService from '../services/UserService';
import AuthService from "../services/AuthService";


export class UserDetailView extends React.Component {

    constructor(props) {
        super(props);
        // console.log('UserDetailView: ', props.match.path)
    }

    componentWillMount(props){
        this.setState({
            loading: true
        });

        if(this.props.match.path === '/profile/:id'){
            let id = this.props.match.params.id;
            UserService.getUser(id).then((data) => {
                // console.log('getUser: ', data);

                this.setState({
                    user: data,
                    loading: false
                });
            }).catch((e) => {
                console.error(e);
            });
        }
        else{
            UserService.getMyProfile().then((data) => {
                this.setState({
                    user: data,
                    loading: false
                });
            }).catch((e) => {
                console.error(e);
            });
        }


    }

    deleteUser(id) {
        UserService.deleteUser(id).then((message) => {
            this.props.history.push('/admin_dashboard');
        }).catch((e) => {
            console.log(e);
        });
    }
    deleteMyProfile() {
        UserService.deleteMyProfile().then((message) => {
            this.props.history.push('/');
        }).catch((e) => {
            console.log(e);
        });
        AuthService.logout();
    }

    render() {
        if (this.state.loading) {
            return (<h2>Loading...</h2>);
        }
        if(AuthService.getRole() === 'Admin'){
            return ( <UserDetail user={this.state.user} onDelete={(id) => this.deleteUser(id)}/>);
        }
        else{
            return ( <UserDetail user={this.state.user} onDelete={() => this.deleteMyProfile()}/>);
        }
    }
}