"use strict";

import React from 'react';

import { UserListLessor } from '../components/UserListLessor';

import UserService from '../services/UserService';


export class UserListLessorView extends React.Component {

    constructor(props) {
        super(props);

        this.state = {
            loading: false,
            data: []
        };
        // this.deleteUser = this.deleteUser.bind(this);
        this.onApprove = this.onApprove.bind(this);
        this.onDeny = this.onDeny.bind(this);

        this.onDetails = this.onDetails.bind(this);
    }

    componentWillMount(){

        this.setState({
            loading: true
        });


        UserService.getLessorRequests().then((data) => {

            this.setState({
                data: [...data],
                loading: false,
                lessor_req: true
            });
        }).catch((e) => {
            console.error(e);
        });

    }

    onApprove(id){

        for(let i=0; i<this.state.data.length; i++){
            if(id === this.state.data[i]._id){
                let user = this.state.data[i];

                user.is_Lessor = true;
                user.lessor_role_requested = false;
                user.role = 'Lessor';

                UserService.grantLessorPrivileges(user).then((data) => {
                    // this.props.history.render();
                    window.location.reload();
                }).catch((e) => {
                    console.error(e);
                    this.setState(Object.assign({}, this.state, {error: 'Error while updating a user'}));
                });


            }

        }
    }


    onDeny(id) {

        for (let i = 0; i < this.state.data.length; i++) {
            if (id === this.state.data[i]._id) {
                let user = this.state.data[i];

                user.is_Lessor = false;
                user.lessor_role_requested = false;


                UserService.grantLessorPrivileges(user).then((data) => {
                    // this.props.history.render();
                    window.location.reload();
                }).catch((e) => {
                    console.error(e);
                    this.setState(Object.assign({}, this.state, {error: 'Error while updating a user'}));
                });


            }
        }
    }

    onDetails(id){

        let value = '/id_doc/'+id.toString();
        console.log(value);
        this.props.history.push(value);

    }

    render() {
        if (this.state.loading) {

            return (<h2>Loading...</h2>);
        }

        return (

            <UserListLessor data={this.state.data} req={this.state.lessor_req} onApprove={(id) => this.onApprove(id)} onDeny={(id) => this.onDeny(id)} onDetails={(id) => this.onDetails(id)} />
        );
    }
}