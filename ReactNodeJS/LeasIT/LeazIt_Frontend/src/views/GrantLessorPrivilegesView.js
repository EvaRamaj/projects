"use strict";

import React from 'react';

import { GrantLessorPrivileges } from '../components/GrantLessorPrivileges';

import UserService from '../services/UserService';


export class GrantLessorPrivilegesView extends React.Component {

    constructor(props) {
        super(props);
    }

    componentWillMount(props){
        this.setState({
            loading: true
        });

        UserService.getUser(this.props.match.params.id).then((data) => {
            this.setState({
                user: data,
                loading: false
            });
        }).catch((e) => {
            console.error(e);
        });
    }

    updateUser(user) {
        console.log('up: ', user);
        UserService.grantLessorPrivileges(user).then((data) => {
            this.props.history.goBack();
        }).catch((e) => {
            console.error(e);
            this.setState(Object.assign({}, this.state, {error: 'Error while updating a user'}));
        });
    }



    render() {
        if (this.state.loading) {
            return (<h2>Loading...</h2>);
        }
        return (<GrantLessorPrivileges user={this.state.user} onClick={(user) => this.updateUser(user)} error={this.state.error} />);

    }
}